#ifndef MYFIFO_H
#define MYFIFO_H

#define FIFOSIZE 1024
#define FIFOSUCCESS 1         // return value on success
#define FIFOFAIL    0         // return value on failure

// RX
void RxFifo_Init(void);
int RxFifo_Put (char data);
int RxFifo_Get (char *datapt);
unsigned short RxFifo_Size (void);

// TX

void TxFifo_Init(void);
int TxFifo_Put (char data);
int TxFifo_Get (char *datapt);
unsigned short TxFifo_Size (void);

#endif
