//
// Created by 田韵豪 on 2023/1/27.
//

#include "time.h"
#include "ds1302.h"

#include <FreeRTOS.h>
#include <FreeRTOS_DNS.h>
#include <core_sntp_client.h>
#include <ctime>


static bool resolveDns(const SntpServerInfo_t *pServerAddr,
                       uint32_t *pIpV4Addr) {
  uint32_t result = FreeRTOS_gethostbyname(pServerAddr->pServerName);
  if (pIpV4Addr)
    *pIpV4Addr = FreeRTOS_ntohl(result);
  return result != 0;
}

struct NetworkContext {
  Socket_t udpSocket;
};

static int32_t UdpTransport_Send(NetworkContext_t *pNetworkContext,
                                 uint32_t serverAddr,
                                 uint16_t serverPort,
                                 const void *pBuffer,
                                 uint16_t bytesToSend) {
  freertos_sockaddr addrInfo = {
      .sin_family = FREERTOS_AF_INET,
      .sin_port = FreeRTOS_htons(serverPort),
      .sin_addr = FreeRTOS_htonl(serverAddr),
  };
  int32_t bytesSent = FreeRTOS_sendto(pNetworkContext->udpSocket,
                              pBuffer,
                              bytesToSend, 0,
                              &addrInfo,
                              sizeof(addrInfo));
  return bytesSent;
}

static int32_t UdpTransport_Recv(NetworkContext_t *pNetworkContext,
                                 uint32_t serverAddr,
                                 uint16_t serverPort,
                                 void *pBuffer,
                                 uint16_t bytesToRecv) {
  freertos_sockaddr addrInfo = {
      .sin_family = FREERTOS_AF_INET,
      .sin_port = FreeRTOS_htons(serverPort),
      .sin_addr = FreeRTOS_htonl(serverAddr),
  };
  socklen_t addrLen = sizeof(addrInfo);

  int32_t bytesReceived = FreeRTOS_recvfrom(pNetworkContext->udpSocket, pBuffer,
                                    bytesToRecv, 0,
                                    &addrInfo,
                                    &addrLen);
  if (bytesReceived == -pdFREERTOS_ERRNO_EWOULDBLOCK)
    return 0;

  return bytesReceived;
}

static void sntpClient_SetTime(const SntpServerInfo_t *pTimeServer,
                               const SntpTimestamp_t *pServerTime,
                               int64_t clockOffsetMs,
                               SntpLeapSecondInfo_t leapSecondInfo) {
  time_t unixSecs;

  if (pServerTime->seconds > 3883978605) {
    unixSecs = (int64_t)pServerTime->seconds - SNTP_TIME_AT_UNIX_EPOCH_SECS;
  } else {
    unixSecs = (int64_t)pServerTime->seconds + UNIX_TIME_SECS_AT_SNTP_ERA_1_SMALLEST_TIME;
  }
  tm tm{};
  ds1302SetTime(localtime_r(&unixSecs, &tm));
}

static void sntpClient_GetTime(SntpTimestamp_t *pCurrentTime) {
  tm tm{};
  ds1302GetTime(&tm);
  pCurrentTime->seconds = mktime(&tm);
  pCurrentTime->fractions = xTaskGetTickCount() % 1000;
}

#define TEST_TIME_SERVER_1                      "ntp.aliyun.com"
#define TEST_TIME_SERVER_2                      "cn.ntp.org.cn"

#define SERVER_RESPONSE_TIMEOUT_MS              3000
#define TIME_REQUEST_SEND_WAIT_TIME_MS          2000
#define TIME_REQUEST_RECEIVE_WAIT_TIME_MS       1000

void timeWork(void*_) {
  uint8_t networkBuffer[SNTP_PACKET_BASE_SIZE];

  NetworkContext_t udpContext = {
      .udpSocket = FreeRTOS_socket(FREERTOS_AF_INET,
                                   FREERTOS_SOCK_DGRAM,
                                   FREERTOS_IPPROTO_UDP)
  };

  // Set non-blocking
  const TickType_t xBlockTime = 0;
  FreeRTOS_setsockopt(udpContext.udpSocket, 0, FREERTOS_SO_RCVTIMEO,
                      &xBlockTime, sizeof(xBlockTime));
  FreeRTOS_setsockopt(udpContext.udpSocket, 0, FREERTOS_SO_SNDTIMEO,
                      &xBlockTime, sizeof(xBlockTime));
  FreeRTOS_bind(udpContext.udpSocket, nullptr, 0);


  SntpServerInfo_t pTimeServers[] =
      {
          {
              .pServerName = TEST_TIME_SERVER_1,
              .serverNameLen = strlen(TEST_TIME_SERVER_1),
              .port = SNTP_DEFAULT_SERVER_PORT,
          },
          {
              .pServerName = TEST_TIME_SERVER_2,
              .serverNameLen = strlen(TEST_TIME_SERVER_2),
              .port = SNTP_DEFAULT_SERVER_PORT,
          }
      };

  UdpTransportInterface_t udpTransportIntf;

  udpTransportIntf.pUserContext = &udpContext;
  udpTransportIntf.sendTo = UdpTransport_Send;
  udpTransportIntf.recvFrom = UdpTransport_Recv;

  SntpContext_t context;
  SntpStatus_t status = Sntp_Init(&context,
                                  pTimeServers,
                                  sizeof(pTimeServers) / sizeof(SntpServerInfo_t),
                                  SERVER_RESPONSE_TIMEOUT_MS,
                                  networkBuffer,
                                  SNTP_PACKET_BASE_SIZE,
                                  resolveDns,
                                  sntpClient_GetTime,
                                  sntpClient_SetTime,
                                  &udpTransportIntf,
                                  NULL);

  configASSERT(status == SntpSuccess);

  while (1) {
    uint32_t rand;
    xApplicationGetRandomNumber(&rand);
    status = Sntp_SendTimeRequest(&context,
                                  rand,
                                  TIME_REQUEST_SEND_WAIT_TIME_MS);
    if (status != SntpSuccess)
      continue;
    status = Sntp_ReceiveTimeResponse(&context, TIME_REQUEST_RECEIVE_WAIT_TIME_MS);
    if (status != SntpSuccess)
      continue;
    vTaskDelay(500 * 1000);
  }
}