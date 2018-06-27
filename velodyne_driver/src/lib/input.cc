/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <velodyne_driver/input.h>
#include <time.h>//加入时间戳转换函数

namespace velodyne_driver
{
  static const size_t packet_size =
    sizeof(velodyne_msgs::VelodynePacket().data);
  static const size_t position_packet_size=554;//定义位置包字节长度（包括报头的42个字节）

  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */
  Input::Input(ros::NodeHandle private_nh, uint16_t port):
    private_nh_(private_nh),
    port_(port)
  {
    private_nh.param("device_ip", devip_str_, std::string(""));
    if (!devip_str_.empty())
      ROS_INFO_STREAM("Only accepting packets from IP address: "
                      << devip_str_);
  }

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   */
  InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port):
    Input(private_nh, port)
  {
    sockfd_ = -1;
    
    if (!devip_str_.empty()) {
      inet_aton(devip_str_.c_str(),&devip_);
    }    

    // connect to Velodyne UDP port
    ROS_INFO_STREAM("Opening UDP socket: port " << port);
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
      {
        perror("socket");               // TODO: ROS_ERROR errno
        return;
      }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(uint16_t (port));          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
  
    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }
  
    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }

    ROS_DEBUG("Velodyne socket fd is %d\n", sockfd_);
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset)
  {
      sockaddr_in sender_address;
      socklen_t sender_address_len = sizeof(sender_address);

      while(true)
      {
          if (!input_available(POLL_TIMEOUT))//判断端口数据包是否已经获取
          {
              return -1;
          }
        // Receive packets that should now be available from the
        // socket using a blocking read.
          ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size,  0,
                (sockaddr*) &sender_address,&sender_address_len);
          if (nbytes < 0)
          {
              if (errno != EWOULDBLOCK)
              {
                  perror("recvfail");
                  ROS_INFO("recvfail");
                  return -1;
              }
          }
          if ((size_t) nbytes == packet_size)break;// read successful

          ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                                   << nbytes << " bytes");
      }
          uint32_t pkttime1=*(uint32_t* )&pkt->data[1200];//取4个字节的时间
          if(pkttime1<pkttime )hourtonsecs+=3.6e12;//判断离上个整点小时过去的秒数是否突然变小，变小意味着又过了一个小时，加上相应的纳秒数
          pkttime=pkttime1;//重新赋值用于下一次判断小时数
          pkt->stamp.fromNSec((uint64_t )pkttime*1e3+hourtonsecs);//把时间戳发到消息的时间戳成员
          return 0;
  }

  int InputSocket::getPositionPacket()//获取位置包数据（即小时数）函数
  {
      sockaddr_in sender_address;
      socklen_t sender_address_len = sizeof(sender_address);
      velodyne_msgs::VelodynePacket *pkt1=new velodyne_msgs::VelodynePacket;//生成新的消息对象指针并初始化

      while(true)
      {
          if (!input_available(POLL_TIMEOUT))//判断端口包是否可以捕获
          {
              return -1;
          }
          // Receive packets that should now be available from the
          // socket using a blocking read.
          ssize_t nbytes = recvfrom(sockfd_, &pkt1->data[0], position_packet_size-42,  0,
                                    (sockaddr*) &sender_address,&sender_address_len);
          if (nbytes < 0)
          {
              if (errno != EWOULDBLOCK)
              {
                  perror("recvfail");
                  ROS_INFO("recvfail");
                  return -1;
              }
          }
          if ((size_t)nbytes==(position_packet_size-42))break;// read successful

          ROS_DEBUG_STREAM("incomplete Velodyne packet read: "
                                   << nbytes << " bytes");
      }

      tm utcdate={0,0,0,0,0,0,0,0,-1};
      utcdate.tm_year=(int)*&pkt1->data[236]-48+10*((int)*&pkt1->data[235]-48)+100;
      utcdate.tm_mon=(int)*&pkt1->data[234]-48+10*((int)*&pkt1->data[233]-48)-1;
      utcdate.tm_mday=(int)*&pkt1->data[232]-48+10*((int)*&pkt1->data[231]-48);//获取年月日
      time_t utcsec=mktime(&utcdate);//将年月日转换为utc时间戳
      uint8_t hour1 =*(uint8_t* )&pkt1->data[213]-48;
      uint8_t hour2 =*(uint8_t* )&pkt1->data[214]-48;//获取小时位
      hourtonsecs=(uint64_t )(hour1*10+hour2)*3.6e12+(uint64_t )utcsec*1e9;//计算出当前整点小时的时间戳
      hour=true;//获取小时位成功并将小时数标志位置1
      return 0;
  }

  bool InputSocket::input_available(int timeout)//判断指定端口是否可以连接
  {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.

        // poll() until input available
        do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);

                if (retval < 0)
                {  // poll() error?
                    if (errno != EINTR)
                    {
                        ROS_ERROR_STREAM("Velodyne port "<< port_ << "poll() error: " << strerror(errno));
                    }
                    return false;
                }

                if (retval == 0)
                { // poll() timeout?
                    ROS_WARN_STREAM("Velodyne port " << port_ << " poll() timeout");
                    return false;
                }

                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
                { // device error?
                    ROS_ERROR_STREAM("Velodyne port " << port_ << "poll() reports Velodyne error");
                    return false;
                }
            } while ((fds[0].revents & POLLIN) == 0);
        return true;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
  InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port,
                       double packet_rate, std::string filename,
                       bool read_once, bool read_fast, double repeat_delay):
    Input(private_nh, port),
    packet_rate_(packet_rate),
    filename_(filename)
  {
    pcap_ = NULL;  
    empty_ = true;

    // get parameters using private node handle
    private_nh.param("read_once", read_once_, false);
    private_nh.param("read_fast", read_fast_, false);
    private_nh.param("repeat_delay", repeat_delay_, 0.0);

    if (read_once_)
      ROS_INFO("Read input file only once.");
    if (read_fast_)
      ROS_INFO("Read input file as quickly as possible.");
    if (repeat_delay_ > 0.0)
      ROS_INFO("Delay %.3f seconds before repeating input file.",
               repeat_delay_);

    // Open the PCAP dump file
    ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
      {
        ROS_FATAL("Error opening Velodyne socket dump file.");
        return;
      }

    std::stringstream filter;
    if( devip_str_ != "" )              // using specific IP?
      {
        filter << "src host " << devip_str_ << " && ";
      }
      filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_,
                 filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  }

  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }

  /** @brief Get one velodyne packet. */
  int InputPCAP::getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    static uint64_t hourtonsecs=0;//当前整点小时转换的时间戳
    static bool hour= false;//获取小时位成功标记
    static uint32_t pkttime=0;//距离上个整点小时过去的微秒数，也用于判断是否又过了一个整点

    while(!hour)//当小时标志位为false时进入循环获取整点小时位时间戳
    {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)//获取pcap文件数据包
        {
            uint16_t position=((uint16_t )*(pkt_data+34)<<8)+(uint16_t)*(pkt_data+35);//获取读到的端口号
            if(position!=POSITION_PORT_NUMBER)
                continue;//不等于8308则继续循环
            if (!devip_str_.empty() &&
                (0 == pcap_offline_filter(&pcap_packet_filter_,
                                          header, pkt_data)))
                continue;

            // Keep the reader from blowing through the file.
            if (read_fast_ == false)
                packet_rate_.sleep();

            memcpy(&pkt->data[0], pkt_data, position_packet_size);//获取数据
            tm utcdate={0,0,0,0,0,0,0,0,-1};
            utcdate.tm_year=(int)*&pkt->data[278]-48+10*((int)*&pkt->data[277]-48)+100;
            utcdate.tm_mon=(int)*&pkt->data[276]-48+10*((int)*&pkt->data[275]-48)-1;
            utcdate.tm_mday=(int)*&pkt->data[274]-48+10*((int)*&pkt->data[273]-48);//获取年月日
            time_t utcsec=mktime(&utcdate);//将年月日转换为utc时间戳
            uint8_t hour1 =*(uint8_t* )&pkt->data[255]-48;
            uint8_t hour2 =*(uint8_t* )&pkt->data[256]-48;//获取当前小时数的十位和个位
            hourtonsecs=(uint64_t )(hour1*10+hour2)*3.6e12+(uint64_t )utcsec*1e9;//计算出当前小时的utc时间戳
            hour=true;//时间标志位值1
            return 0;
        }
    }

    while (hour)//当取到小时位时进入循环获取当前时间utc时间戳
      {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
          {
            // Skip packets not for the correct port and from the
            // selected IP address.

            uint16_t position=((uint16_t )*(pkt_data+34)<<8)+(uint16_t)*(pkt_data+35);
            if(position!=DATA_PORT_NUMBER)
                continue;//端口好不等于2368则继续循环
            if (!devip_str_.empty() &&
                (0 == pcap_offline_filter(&pcap_packet_filter_,
                                          header, pkt_data)))
              continue;

            // Keep the reader from blowing through the file.
            if (read_fast_ == false)
              packet_rate_.sleep();

            memcpy(&pkt->data[0], pkt_data+42, packet_size);

            uint32_t pkttime1=*(uint32_t* )&pkt->data[1200];//获取距离上一个整点小时的时间戳
            if(pkttime1<pkttime )hourtonsecs+=3.6e12;//判断离上个整点小时过去的秒数是否突然变小，变小意味着又过了一个小时，加上相应的纳秒数
            pkttime=pkttime1;//重新赋值用于下一次判断小时数
            pkt->stamp.fromNSec((uint64_t )pkttime*1e3+hourtonsecs);//把时间戳发到消息的时间戳成员
            empty_ = false;
            return 0;                   // success
          }

        if (empty_)                 // no data in file?
          {
            ROS_WARN("Error %d reading Velodyne packet: %s",
                     res, pcap_geterr(pcap_));
            return -1;
          }

        if (read_once_)
          {
            ROS_INFO("end of file reached -- done reading.");
            return -1;
          }

        if (repeat_delay_ > 0.0)
          {
            ROS_INFO("end of file reached -- delaying %.3f seconds.",
                     repeat_delay_);
            usleep(rint(repeat_delay_ * 1000000.0));
          }

        ROS_DEBUG("replaying Velodyne dump file");

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_ = true;              // maybe the file disappeared?
      } // loop back and try again
  }

  int InputPCAP::getPositionPacket()
  {  }

} // velodyne namespace
