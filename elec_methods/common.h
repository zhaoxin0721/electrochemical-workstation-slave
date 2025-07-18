#include <stdint.h>
#include <stdbool.h>

// typedef void (* send_fac_stream_t) (uint8_t * data, uint8_t len);
typedef void (* loop_method_t)();

/* size of stack area used by each thread */
#define STACKSIZE 4096

/* scheduling priority used by each thread */
#define PRIORITY 3

int elec_method_commit(loop_method_t loop_method);

// 缓冲发送，确保蓝牙有足够的时间处理
void send_fac_stream(uint8_t * data, uint8_t len);

// 直接发送，不经过缓冲队列，需要自行错误处理
int send_fac_stream_directly(uint8_t * data, uint8_t len);

// 大队列缓冲，针对短时大数据
void commit_fac_stream(uint8_t * data, uint8_t len, bool append);

// code

#define CODE_OCP  0x02
#define CODE_EIS  0x03
#define CODE_CV   0x04
#define CODE_CA   0x05

