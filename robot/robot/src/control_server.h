#ifndef CONTROL_SERVER_H
#define CONTROL_SERVER_H
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
//#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>
using namespace std;
#define PORT 10001

extern int control_server();
#endif // CONTROL_SERVER_H

