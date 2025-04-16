#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>  // For gettimeofday()
#include <time.h>      // For time_t, struct tm, time()

// Flag to control program execution
volatile bool running = true;

// Signal handler for clean termination
void sig_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        running = false;
        printf("\nReceived termination signal. Shutting down...\n");
    }
}

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace sl;

// UDP Publisher settings
struct UdpPublisher {
    bool enabled;
    int socket_fd;
    struct sockaddr_in dest_addr;
    char dest_ip[64];
    int dest_port;
};

UdpPublisher publisher = {false, -1, {0}, {0}, 0};

bool init_udp_publisher(const char* ip, int port) {
    // Create socket
    publisher.socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (publisher.socket_fd < 0) {
        fprintf(stderr, "Failed to create socket\n");
        return false;
    }

    // Configure destination address
    memset(&publisher.dest_addr, 0, sizeof(publisher.dest_addr));
    publisher.dest_addr.sin_family = AF_INET;
    publisher.dest_addr.sin_port = htons(port);
    
    inet_pton(AF_INET, ip, &publisher.dest_addr.sin_addr);

    strncpy(publisher.dest_ip, ip, sizeof(publisher.dest_ip)-1);
    publisher.dest_port = port;
    publisher.enabled = true;
    
    printf("UDP Publisher initialized. Publishing to %s:%d\n", ip, port);
    return true;
}

void close_udp_publisher() {
    if (publisher.socket_fd >= 0) {
        close(publisher.socket_fd);
        publisher.socket_fd = -1;
    }
    publisher.enabled = false;
}

void send_scan_data_udp(sl_lidar_response_measurement_node_hq_t* nodes, size_t count) {
    if (!publisher.enabled || publisher.socket_fd < 0) {
        return;
    }
    
    // Pack the data: timestamp (8 bytes) + count (4 bytes) + data (16 bytes per point)
    // Each point: angle (4 bytes) + distance (4 bytes) + quality (4 bytes) + flag (4 bytes)
    size_t buffer_size = 12 + (count * 16);
    char* buffer = (char*)malloc(buffer_size);
    
    if (!buffer) {
        fprintf(stderr, "Failed to allocate memory for UDP packet\n");
        return;
    }
    
    // Add timestamp (use current time in milliseconds since epoch)
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t timestamp = (tv.tv_sec * 1000ULL) + (tv.tv_usec / 1000ULL);
    
    memcpy(buffer, &timestamp, 8);
    
    // Add count
    uint32_t count32 = (uint32_t)count;
    memcpy(buffer + 8, &count32, 4);
    
    // Add scan points
    char* ptr = buffer + 12;
    for (size_t i = 0; i < count; i++) {
        float angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        float dist = nodes[i].dist_mm_q2 / 4.0f;
        uint32_t quality = nodes[i].quality;
        uint32_t flag = nodes[i].flag;
        
        memcpy(ptr, &angle, 4);
        memcpy(ptr + 4, &dist, 4);
        memcpy(ptr + 8, &quality, 4);
        memcpy(ptr + 12, &flag, 4);
        
        ptr += 16;
    }
    
    // Send the data
    sendto(publisher.socket_fd, buffer, buffer_size, 0, 
           (struct sockaddr*)&publisher.dest_addr, sizeof(publisher.dest_addr));
    
    free(buffer);
}

void print_usage(int argc, const char * argv[])
{
    printf("Continuous 360° LIDAR data grabber for SLAMTEC LIDAR with UDP publishing.\n"
           "Version:  %s \n"
           "Usage:\n"
           " For serial channel %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
           " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
           "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089.\n"
           " --publish <destination ip> <destination port> : Required for UDP publishing\n"
           , SL_LIDAR_SDK_VERSION, argv[0], argv[0]);
}

sl_result continuous_360_scan_and_publish(ILidarDriver * drv)
{
    sl_result ans;
    
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    printf("Starting continuous 360° scan mode. Press Ctrl+C to stop...\n");

    bool is_360_complete = false;
    float last_angle = 0.0f;
    float current_angle = 0.0f;
    
    // Temporary storage for a complete 360° scan
    sl_lidar_response_measurement_node_hq_t full_scan[8192];
    size_t full_scan_count = 0;
    
    while (running) {
        ans = drv->grabScanDataHq(nodes, count, 0);
        if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
            // Sort the scan data by angle
            drv->ascendScanData(nodes, count);
            
            for (size_t i = 0; i < count; i++) {
                current_angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                
                // Check if we've completed a 360° scan (when angle wraps around from 359° to 0°)
                if (!is_360_complete && last_angle > 270.0f && current_angle < 90.0f) {
                    is_360_complete = true;
                    
                    // Only publish if we have scan data
                    if (full_scan_count > 0 && publisher.enabled) {
                        send_scan_data_udp(full_scan, full_scan_count);
                        printf("Published complete 360° scan with %zu points to %s:%d\n", 
                               full_scan_count, publisher.dest_ip, publisher.dest_port);
                    }
                    
                    // Reset for next scan
                    full_scan_count = 0;
                }
                
                // If we're still in the middle of collecting a 360° scan, add this point
                if (!is_360_complete) {
                    if (full_scan_count < _countof(full_scan)) {
                        memcpy(&full_scan[full_scan_count++], &nodes[i], sizeof(nodes[i]));
                    }
                } else {
                    // Start new scan
                    memcpy(&full_scan[0], &nodes[i], sizeof(nodes[i]));
                    full_scan_count = 1;
                    is_360_complete = false;
                }
                
                last_angle = current_angle;
            }
        } else {
            printf("Error grabbing scan data: %x\n", ans);
            break;
        }
        
        delay(5); // Small delay to prevent CPU overload
    }

    return ans;
}

int main(int argc, const char * argv[]) {
    const char *opt_channel = NULL;
    const char *opt_channel_param_first = NULL;
    sl_u32      opt_channel_param_second = 0;
    sl_result   op_result;
    int         opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    
    const char *publish_ip = NULL;
    int         publish_port = 0;

    IChannel* _channel;

    // Set up signal handling for clean termination
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    if (argc < 5) {
        print_usage(argc, argv);
        return -1;
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--channel") == 0) {
            if (i + 1 < argc) {
                opt_channel = argv[++i];
                if (strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0) {
                    if (i + 1 < argc) {
                        opt_channel_param_first = argv[++i];
                        if (i + 1 < argc && argv[i+1][0] != '-') {
                            opt_channel_param_second = strtoul(argv[++i], NULL, 10);
                        }
                    }
                } else if (strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0) {
                    if (i + 1 < argc) {
                        opt_channel_param_first = argv[++i];
                        if (i + 1 < argc && argv[i+1][0] != '-') {
                            opt_channel_param_second = strtoul(argv[++i], NULL, 10);
                        }
                    }
                    opt_channel_type = CHANNEL_TYPE_UDP;
                } else {
                    print_usage(argc, argv);
                    return -1;
                }
            }
        } else if (strcmp(argv[i], "--publish") == 0) {
            if (i + 2 < argc) {
                publish_ip = argv[++i];
                publish_port = atoi(argv[++i]);
            } else {
                printf("Error: --publish requires IP and port arguments\n");
                print_usage(argc, argv);
                return -1;
            }
        }
    }

    if (!opt_channel_param_first) {
        print_usage(argc, argv);
        return -1;
    }

    // Initialize UDP publisher - required for this application
    if (publish_ip && publish_port > 0) {
        if (!init_udp_publisher(publish_ip, publish_port)) {
            fprintf(stderr, "Failed to initialize UDP publisher\n");
            return -1;
        }
    } else {
        fprintf(stderr, "UDP publishing configuration (--publish) is required\n");
        print_usage(argc, argv);
        return -1;
    }

    // create the driver instance
    ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "Insufficient memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_health_t healthinfo;
    sl_lidar_response_device_info_t devinfo;
    do {
        // try to connect
        if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
        }
        else if (opt_channel_type == CHANNEL_TYPE_UDP) {
            _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        }
        
        if (SL_IS_FAIL((drv)->connect(_channel))) {
            switch (opt_channel_type) {    
                case CHANNEL_TYPE_SERIALPORT:
                    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                        , opt_channel_param_first);
                    break;
                case CHANNEL_TYPE_UDP:
                    fprintf(stderr, "Error, cannot connect to the ip addr %s with the udp port %u.\n"
                        , opt_channel_param_first, opt_channel_param_second);
                    break;
                default:
                    fprintf(stderr, "Error, cannot connect to the specified channel.\n");
                    break;
            }
            break;
        }

        // retrieving the device info
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                fprintf(stderr, "Error, operation time out.\n");
            } else {
                fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
            }
            break;
        }

        // print out the device serial number, firmware and hardware version number
        printf("SLAMTEC LIDAR S/N: ");
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
        }

        printf("\n"
                "Version:  %s \n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , SL_LIDAR_SDK_VERSION
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);

        // check the device health
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) {
            printf("Lidar health status : ");
            switch (healthinfo.status) 
            {
                case SL_LIDAR_STATUS_OK:
                    printf("OK.");
                    break;
                case SL_LIDAR_STATUS_WARNING:
                    printf("Warning.");
                    break;
                case SL_LIDAR_STATUS_ERROR:
                    printf("Error.");
                    break;
            }
            printf(" (errorcode: %d)\n", healthinfo.error_code);

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            break;
        }

        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            break;
        }

        // Start motor if using serial port
        if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
            drv->setMotorSpeed();
        }

        // Start scan operation
        if (SL_IS_FAIL(drv->startScan(0, 1))) // force scan operation regardless of motor rotation
        {
            fprintf(stderr, "Error, cannot start the scan operation.\n");
            break;
        }

        // Start continuous 360° scan and publish
        if (SL_IS_FAIL(continuous_360_scan_and_publish(drv))) {
            fprintf(stderr, "Error during continuous scan.\n");
            break;
        }

    } while(0);

    // Clean up
    drv->stop();
    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        delay(20);
        drv->setMotorSpeed(0);
    }
    
    if (publisher.enabled) {
        close_udp_publisher();
    }
    
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}