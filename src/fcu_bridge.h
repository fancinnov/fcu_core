#include <stdio.h>

typedef struct {
	uint8_t* pBuff;
	uint8_t* pEnd;  // pBuff + legnth
	uint8_t* wp;    // Write Point
	uint8_t* rp;    // Read Point
	uint16_t length;
	uint8_t  flagOverflow; // set when buffer overflowed
} RingBuffer;

static bool send_lock=false;
static bool receive_lock=false;

bool get_send_lock(void){
    return send_lock;
}

bool get_receive_lock(void){
    return receive_lock;
}

void set_send_lock(bool lock){
    send_lock=lock;
}

void set_receive_lock(bool lock){
    receive_lock=lock;
}

//初始化RingBuffer
void rbInit(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length)
{
	pRingBuff->pBuff = buff;
	pRingBuff->pEnd  = buff + length;
	pRingBuff->wp = buff;
	pRingBuff->rp = buff;
	pRingBuff->length = length;
	pRingBuff->flagOverflow = 0;
}

//清空RingBuffer
inline void rbClear(RingBuffer* pRingBuff)
{
 	pRingBuff->wp = pRingBuff->pBuff;
	pRingBuff->rp = pRingBuff->pBuff;
	pRingBuff->flagOverflow = 0;
}

//把一字节数据存进RingBuffer
inline void rbPush(RingBuffer* pRingBuff, uint8_t value)
{
	uint8_t* wp_next = pRingBuff->wp + 1;
	if( wp_next == pRingBuff->pEnd ) {
		wp_next -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	if( wp_next != pRingBuff->rp ) {
		*pRingBuff->wp = value;
		pRingBuff->wp = wp_next;
	} else {
		pRingBuff->flagOverflow = 1;
	}
}

//读取一个字节数据出来
inline uint8_t rbPop(RingBuffer* pRingBuff)
{
	if( pRingBuff->rp == pRingBuff->wp )
		return 0; // empty

	uint8_t ret = *pRingBuff->rp;
	pRingBuff->rp++;
	if( pRingBuff->rp == pRingBuff->pEnd ) {
		pRingBuff->rp -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	return ret;
}

//当前还有多少数据没被读取
inline uint16_t rbGetCount(const RingBuffer* pRingBuff)
{
	return (pRingBuff->wp - pRingBuff->rp + pRingBuff->length) % pRingBuff->length;
}

//当前RingBuffer是不是空的
inline int8_t rbIsEmpty(const RingBuffer* pRingBuff)
{
	return pRingBuff->wp == pRingBuff->rp;
}

//当前RingBuffer是不是满的
inline int8_t rbIsFull(const RingBuffer* pRingBuff)
{
 	return (pRingBuff->rp - pRingBuff->wp + pRingBuff->length - 1) % pRingBuff->length == 0;
}
