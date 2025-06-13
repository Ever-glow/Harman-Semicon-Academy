#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include <stdint.h>
#include <sleep.h>

// AXI-SPI Master 레지스터 오프셋
#define SPI_BASE     XPAR_MYIP_AXI_SPI_M_0_S00_AXI_BASEADDR
#define REG_CR       0x00    // [3]=SW_SEL, [2]=CPOL, [1]=CPHA, [0]=START
#define REG_SOD      0x04    // Master→Slave 데이터
#define REG_SID      0x08    // Slave→Master→ 데이터
#define REG_SR       0x0C    // [1]=READY, [0]=DONE

#define CR_START     (1<<0)
#define SR_DONE      (1<<0)
#define SR_READY_BIT  (1 << 1)

// 8bit 읽기/쓰기 헬퍼
static inline uint8_t  rd_sr(void)  { return Xil_In32(SPI_BASE+REG_SR)  & 0xFF; }
static inline void     wr_cr(uint8_t v) { Xil_Out32(SPI_BASE+REG_CR,  v); }
static inline void     wr_sod(uint8_t v){ Xil_Out32(SPI_BASE+REG_SOD, v); }
static inline uint32_t read_sid(void) { return Xil_In32(SPI_BASE + REG_SID); }

// START 펄스만! (SW_SEL=0, CPOL=0, CPHA=0)
static void spi_start_pulse(uint8_t data){
    wr_sod(data);
    wr_cr(0x01);
    wr_cr(0x08);
}

// DONE ↑↓ 대기
static void wait_done_fall(){
    while((rd_sr() & SR_DONE));   // ↑
   while(!(rd_sr() & SR_DONE));    // ↓
}

int main(){
    xil_printf("\r\n--- SPI: WRITE MODE 4-BYTE ONLY ---\r\n");

    while(1) {
    	while((rd_sr() & SR_READY_BIT)) {
			xil_printf(">> CMD=0x80 (WRITE mode)\n");
			spi_start_pulse(0x80);
			wait_done_fall();
    	}
    	spi_start_pulse(0x00);
    }

    return 0;
}
