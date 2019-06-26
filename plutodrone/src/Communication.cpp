#include "ros/ros.h"
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <plutodrone/Communication.h>
#include <plutodrone/Protocol.h>

using namespace std;

const int PORT = 23;
const char* IP_ADDRESS = "192.168.4.1";
const int CAMERA_PORT = 9060;
const char* CAMERA_IP_ADDRESS = "192.168.0.1";

Protocol pro;
int indx=0;
unsigned int len = 0;
uint8_t checksum=0;
uint8_t command=0;
uint8_t payload_size=0;

int optval;
socklen_t optlen = sizeof(optval);

int socketSyckLock=0;
int socketOpStarted=0;
int checksumIndex=0;
uint8_t recbuf[1024];

int c_state = IDLE;
uint8_t c;
bool err_rcvd = false;
int offset = 0, dataSize = 0;
uint8_t cmd;
int i = 0;

bool Communication::connectSock(){
  int res;
  struct sockaddr_in addr;
  long arg;
  fd_set myset;
  struct timeval tv;
  int valopt;
  socklen_t lon;

  cout<<"Connecting to Pluto......\n";

  // Create socket
  sockID = socket(AF_INET, SOCK_STREAM, 0);
  //Check if socket is created. If not, socket() returns -1
  if (sockID < 0) {
     cout<<"Cannot connect to Pluto, please try again1\n";
     exit(0);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(CAMERA_PORT);
  addr.sin_addr.s_addr = inet_addr(CAMERA_IP_ADDRESS);

  //socket() sets it to blocking
  // Set to non-blocking.
  if( (arg = fcntl(sockID, F_GETFL, NULL)) < 0) {
    cout<<"Cannot connect to Pluto, please try again2\n";
     exit(0);
  }
  arg |= O_NONBLOCK;
  if( fcntl(sockID, F_SETFL, arg) < 0) {
    cout<<"Cannot connect to Pluto, please try again3\n";
    exit(0);
  }

  // Trying to connect with timeout
  res = connect(sockID, (struct sockaddr *)&addr, sizeof(addr));
  if (res < 0) {
     if (errno == EINPROGRESS) {
      do {
        tv.tv_sec = 7;
        tv.tv_usec = 0;
        FD_ZERO(&myset);
        FD_SET(sockID, &myset);
        res = select(sockID+1, NULL, &myset, NULL, &tv);
        if (res < 0 && errno != EINTR) {
          cout<<"Cannot connect to Pluto, please try again4\n";
          exit(0);
        }
        else if (res > 0) {
          // Socket selected for write
          lon = sizeof(int);
          if (getsockopt(sockID, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon) < 0) {
                 cout<<"Cannot connect to Pluto, please try again5\n";
                 exit(0);
              }
              // Check the value returned...
              if (valopt) {
                cout<<"Cannot connect to Pluto, please try again6\n";
                 exit(0);
              }
              break;
           }
           else {
             cout<<"Cannot connect to Pluto, please try again7\n";
              exit(0);
           }
        } while (1);
     }
     else {
         cout<<"Cannot connect to Pluto, please try again8\n";
        exit(0);
     }
  }

  // Set to blocking mode again...
  if( (arg = fcntl(sockID, F_GETFL, NULL)) < 0) {
     cout<<"Cannot connect to Pluto, please try again9\n";
      exit(0);
  }
  arg &= (~O_NONBLOCK);
  if( fcntl(sockID, F_SETFL, arg) < 0) {
     cout<<"Cannot connect to Pluto, please try again0\n";
     exit(0);
  }


  /* Check the status for the keepalive option */
  if(getsockopt(sockID, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
    cout<<"Cannot connect to Pluto, please try again11\n";
    close(sockID);
    exit(EXIT_FAILURE);
  }

  /* Set the option active */
  optval = 1;
  optlen = sizeof(optval);
  if(setsockopt(sockID, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
      cout<<"Cannot connect to Pluto, please try again12\n";
      close(sockID);
      exit(EXIT_FAILURE);
  }

  /* Check the status again */
  if(getsockopt(sockID, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
      cout<<"Cannot connect to Pluto, please try again13\n";
      close(sockID);
      exit(EXIT_FAILURE);
  }

  int error = 0;
  socklen_t len = sizeof (error);
  int retval = getsockopt (sockID, SOL_SOCKET, SO_ERROR, &error, &len);

  if (retval != 0) {
    cout<<"Cannot connect to Pluto, please try again14\n";
    exit(EXIT_FAILURE);
  }

  if (error != 0) {
    cout<<"Cannot connect to Pluto, please try again15\n";
    exit(EXIT_FAILURE);
  }else{
    cout<<"Pluto Connected\n";
  }
  return true;
}

int Communication::writeSock(const void *buf, int count){
  int k=write(sockID,buf,count);
  socketSyckLock=1;
  return k;
}

uint8_t Communication::readSock(void *buf, int count){
  int k=read(sockID,buf,count);
  if(k>0){
    uint8_t val=recbuf[0];
    return val;
  }else{
    return k;
  }
}

void Communication::readFrame(){
  c = readSock(recbuf,1);
  if (c_state == IDLE){
    c_state = (c == '$') ? HEADER_START : IDLE;
  }else if (c_state == HEADER_START) {
    c_state = (c == 'M') ? HEADER_M : IDLE;
  }else if (c_state == HEADER_M) {
    if (c == '>') {
      c_state = HEADER_ARROW;
    } else if (c == '!') {
      c_state = HEADER_ERR;
    } else {
      c_state = IDLE;
    }
  } else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
    /* is this an error message? */
    err_rcvd = (c_state == HEADER_ERR);
    dataSize = (c & 0xFF);
    offset = 0;
    checksum = 0;
    checksum ^= (c & 0xFF);
    /* the command is to follow */
    c_state = HEADER_SIZE;
  }else if (c_state == HEADER_SIZE) {
    cmd = (uint8_t) (c & 0xFF);
    checksum ^= (c & 0xFF);
    c_state = HEADER_CMD;
  }else if (c_state == HEADER_CMD && offset < dataSize) {
    checksum ^= (c & 0xFF);
    inputBuffer[offset++] = (uint8_t) (c & 0xFF);
  }else if (c_state == HEADER_CMD && offset >= dataSize) {
    /* compare calculated and transferred checksum */
    if ((checksum & 0xFF) == (c & 0xFF)) {
      if (err_rcvd) {
      }else {
        bufferIndex=0;
        pro.evaluateCommand(cmd);
      }
    }else {

    }
    c_state = IDLE;
  }
}

bool Communication::connectMulSock(const std::string& ip, int index){

  int res;
  struct sockaddr_in addr;
  long arg;
  fd_set myset;
  struct timeval tv;
  int valopt;
  socklen_t lon;

  cout<<"Connecting to Pluto\n";

  // Create interface
  sockIDList[index] = socket(AF_INET, SOCK_STREAM, 0);
  //Check if socket is created. If not, socket() returns -1
  if (sockIDList[index] < 0) {
    // fprintf(stderr, "Error creating socket (%d %s)\n", errno, strerror(errno));
     cout<<"Cannot connect to Pluto, please try again1\n";
     exit(0);
  }

  //address of the server
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = inet_addr(ip.c_str());

  //socket() sets it to blocking
  // Set to non-blocking. arg will re
  if( (arg = fcntl(sockIDList[index], F_GETFL, NULL)) < 0) {
    cout<<"Cannot connect to Pluto, please try again2\n";
     exit(0);
  }
  arg |= O_NONBLOCK;
  if( fcntl(sockIDList[index], F_SETFL, arg) < 0) {
    cout<<"Cannot connect to Pluto, please try again3\n";
    exit(0);
  }

  // Trying to connect with timeout. connect() is blocking
  res = connect(sockIDList[index], (struct sockaddr *)&addr, sizeof(addr));

  if (res < 0) {
     if (errno == EINPROGRESS) {
      do {
        tv.tv_sec = 7;
        tv.tv_usec = 0;
        FD_ZERO(&myset);
        FD_SET(sockIDList[index], &myset);
        res = select(sockIDList[index]+1, NULL, &myset, NULL, &tv);

        if (res < 0 && errno != EINTR) {
          cout<<"Cannot connect to Pluto, please try again4\n";
          exit(0);
        }else if (res > 0) {
          // Socket selected for write
          lon = sizeof(int);
          if (getsockopt(sockIDList[index], SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon) < 0) {
            cout<<"Cannot connect to Pluto, please try again5\n";
            exit(0);
          }
          // Check the value returned...
          if (valopt) {
            cout<<"Cannot connect to Pluto, please try again6\n";
            exit(0);
          }
          break;
        }else {
          cout<<"Cannot connect to Pluto, please try again7\n";
          exit(0);
        }
      }while (1);
     }
     else {
         cout<<"Cannot connect to Pluto, please try again8\n";
        exit(0);
     }
  }

  // Set to blocking mode again...
  if( (arg = fcntl(sockIDList[index], F_GETFL, NULL)) < 0) {
     cout<<"Cannot connect to Pluto, please try again9\n";
      exit(0);
  }
  arg &= (~O_NONBLOCK);
  if( fcntl(sockIDList[index], F_SETFL, arg) < 0) {
     cout<<"Cannot connect to Pluto, please try again10\n";
     exit(0);
  }

  /* Check the status for the keepalive option */
  if(getsockopt(sockIDList[index], SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
    cout<<"Cannot connect to Pluto, please try again11\n";
    close(sockIDList[index]);
    exit(EXIT_FAILURE);
  }

  optval = 1;
  optlen = sizeof(optval);
  if(setsockopt(sockIDList[index], SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
      cout<<"Cannot connect to Pluto, please try again12\n";
      close(sockIDList[index]);
      exit(EXIT_FAILURE);
  }

  if(getsockopt(sockIDList[index], SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
      cout<<"Cannot connect to Pluto, please try again13\n";
      close(sockIDList[index]);
      exit(EXIT_FAILURE);
  }

  int error = 0;
  socklen_t len = sizeof (error);
  int retval = getsockopt (sockIDList[index], SOL_SOCKET, SO_ERROR, &error, &len);

  if (retval != 0) {
    cout<<"Cannot connect to Pluto, please try again14\n";
    exit(EXIT_FAILURE);
  }

  if (error != 0) {
    cout<<"Cannot connect to Pluto, please try again15\n";
    exit(EXIT_FAILURE);
  }else{
    cout<<"Pluto Connected\n";
  }
  return true;
}

int Communication::writeMulSock(const void *buf, int count, int i){
  int k=write(sockIDList[i],buf,count);
  socketSyckLock=1;
  return k;
}

uint8_t Communication::readMulSock(void *buf, int count, int index){
  int k=read(sockIDList[index],buf,count);
  if(k>0){
    uint8_t val=recbuf[0];
    return val;
  }else{
    return k;
  }
}

void Communication::readMulFrame(int index){
  cout<<index;
  c = readMulSock(recbuf,1,index);

  if (c_state == IDLE){
    c_state = (c == '$') ? HEADER_START : IDLE;
  }else if (c_state == HEADER_START) {
    c_state = (c == 'M') ? HEADER_M : IDLE;
  }else if (c_state == HEADER_M) {
    if (c == '>') {
      c_state = HEADER_ARROW;
    } else if (c == '!') {
      c_state = HEADER_ERR;
    } else {
      c_state = IDLE;
    }
  } else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
    /* is this an error message? */
    err_rcvd = (c_state == HEADER_ERR);
    /* now we are expecting the payload size */
    dataSize = (c & 0xFF);
    /* reset index variables */
    //  p = 0;
    offset = 0;
    checksum = 0;
    checksum ^= (c & 0xFF);
    /* the command is to follow */
    c_state = HEADER_SIZE;
  }else if (c_state == HEADER_SIZE) {
    cmd = (uint8_t) (c & 0xFF);
    checksum ^= (c & 0xFF);
    c_state = HEADER_CMD;
  }else if (c_state == HEADER_CMD && offset < dataSize) {
    checksum ^= (c & 0xFF);
    inputBuffer[offset++] = (uint8_t) (c & 0xFF);
  }else if (c_state == HEADER_CMD && offset >= dataSize) {
    /* compare calculated and transferred checksum */
    if ((checksum & 0xFF) == (c & 0xFF)) {
      if (err_rcvd) {

      }else {
        bufferIndex=0;
        cout<<cmd;
      }
    }else
    {

    }
    c_state = IDLE;
  }
  // cout<<c_state;
}
