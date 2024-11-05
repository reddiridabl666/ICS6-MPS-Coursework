#include "enc28j60.h"
#include "uart.h"

/*
 * SPI
 */

uint8_t enc28j60_current_bank[2] = {0, 0};
uint16_t enc28j60_rxrdpt[2] = {0, 0};

#define enc28j60_select(cs) ENC28J60_SPI_PORT &= ~(1<<cs)
#define enc28j60_release(cs) ENC28J60_SPI_PORT |= (1<<cs)

uint8_t enc28j60_rxtx(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)))
		;
	return SPDR;
}

#define enc28j60_rx() enc28j60_rxtx(0xff)
#define enc28j60_tx(data) enc28j60_rxtx(data)

// Generic SPI read command
uint8_t enc28j60_read_op(uint8_t cs, uint8_t cmd, uint8_t adr)
{
	uint8_t data;

	enc28j60_select(cs);
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	if(adr & 0x80) // throw out dummy byte 
		enc28j60_rx(); // when reading MII/MAC register
	data = enc28j60_rx();
	enc28j60_release(cs);
	return data;
}

// Generic SPI write command
void enc28j60_write_op(uint8_t cs, uint8_t cmd, uint8_t adr, uint8_t data)
{
	enc28j60_select(cs);
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	enc28j60_tx(data);
	enc28j60_release(cs);
}

// Initiate software reset
void enc28j60_soft_reset(uint8_t cs)
{
	enc28j60_select(cs);
	enc28j60_tx(ENC28J60_SPI_SC);
	enc28j60_release(cs);
	
	enc28j60_current_bank[cs-1] = 0;
	_delay_ms(1); // Wait until device initializes
}


/*
 * Memory access
 */

// Set register bank
void enc28j60_set_bank(uint8_t cs, uint8_t adr)
{
	uint8_t bank;

	if( (adr & ENC28J60_ADDR_MASK) < ENC28J60_COMMON_CR )
	{
		bank = (adr >> 5) & 0x03; //BSEL1|BSEL0=0x03
		if(bank != enc28j60_current_bank[cs-1])
		{
			enc28j60_write_op(cs, ENC28J60_SPI_BFC, ECON1, 0x03);
			enc28j60_write_op(cs, ENC28J60_SPI_BFS, ECON1, bank);
			enc28j60_current_bank[cs-1] = bank;
		}
	}
}

// Read register
uint8_t enc28j60_rcr(uint8_t cs, uint8_t adr)
{
	enc28j60_set_bank(cs, adr);
	return enc28j60_read_op(cs, ENC28J60_SPI_RCR, adr);
}

// Read register pair
uint16_t enc28j60_rcr16(uint8_t cs, uint8_t adr)
{
	enc28j60_set_bank(cs, adr);
	return enc28j60_read_op(cs, ENC28J60_SPI_RCR, adr) |
		(enc28j60_read_op(cs, ENC28J60_SPI_RCR, adr+1) << 8);
}

// Write register
void enc28j60_wcr(uint8_t cs, uint8_t adr, uint8_t arg)
{
	enc28j60_set_bank(cs, adr);
	enc28j60_write_op(cs, ENC28J60_SPI_WCR, adr, arg);
}

// Write register pair
void enc28j60_wcr16(uint8_t cs, uint8_t adr, uint16_t arg)
{
	enc28j60_set_bank(cs, adr);
	enc28j60_write_op(cs, ENC28J60_SPI_WCR, adr, arg);
	enc28j60_write_op(cs, ENC28J60_SPI_WCR, adr+1, arg>>8);
}

// Clear bits in register (reg &= ~mask)
void enc28j60_bfc(uint8_t cs, uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(cs, adr);
	enc28j60_write_op(cs, ENC28J60_SPI_BFC, adr, mask);
}

// Set bits in register (reg |= mask)
void enc28j60_bfs(uint8_t cs, uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(cs, adr);
	enc28j60_write_op(cs, ENC28J60_SPI_BFS, adr, mask);
}

// Read Rx/Tx buffer (at ERDPT)
void enc28j60_read_buffer(uint8_t cs, uint8_t *buf, uint16_t len)
{
	enc28j60_select(cs);
	enc28j60_tx(ENC28J60_SPI_RBM);
	while(len--)
		*(buf++) = enc28j60_rx();
	enc28j60_release(cs);
}

// Write Rx/Tx buffer (at EWRPT)
void enc28j60_write_buffer(uint8_t cs, uint8_t *buf, uint16_t len)
{
	enc28j60_select(cs);
	enc28j60_tx(ENC28J60_SPI_WBM);
	while(len--)
		enc28j60_tx(*(buf++));
	enc28j60_release(cs);
}

// Read PHY register
uint16_t enc28j60_read_phy(uint8_t cs, uint8_t adr)
{
	enc28j60_wcr(cs, MIREGADR, adr);
	enc28j60_bfs(cs, MICMD, MICMD_MIIRD);
	while(enc28j60_rcr(cs, MISTAT) & MISTAT_BUSY)
		;
	enc28j60_bfc(cs, MICMD, MICMD_MIIRD);
	return enc28j60_rcr16(cs, MIRD);
}

// Write PHY register
void enc28j60_write_phy(uint8_t cs, uint8_t adr, uint16_t data)
{
	enc28j60_wcr(cs, MIREGADR, adr);
	enc28j60_wcr16(cs, MIWR, data);
	while(enc28j60_rcr(cs, MISTAT) & MISTAT_BUSY)
		;
}


/*
 * Init & packet Rx/Tx
 */

void enc28j60_init(uint8_t cs)
{
	// Initialize SPI
	enc28j60_release(cs);

	// Reset ENC28J60
	enc28j60_soft_reset(cs);
    enc28j60_bfc(cs, ECON1, ECON1_RXEN);


	// Setup Rx/Tx buffer
	enc28j60_wcr16(cs, ERXST, ENC28J60_RXSTART);
	enc28j60_wcr16(cs, ERXRDPT, ENC28J60_RXSTART);
	enc28j60_wcr16(cs, ERXND, ENC28J60_RXEND);
	enc28j60_rxrdpt[cs-1] = ENC28J60_RXSTART;

    enc28j60_wcr(cs, ERXFCON, 0x40); // accept all packets

    // while (!(enc28j60_rcr(cs, ESTAT) & ESTAT_CLKRDY)) {}

	// Setup MAC
	enc28j60_wcr(cs, MACON1, MACON1_TXPAUS| // Enable flow control
		MACON1_RXPAUS|MACON1_MARXEN); // Enable MAC Rx
	enc28j60_wcr(cs, MACON2, 0); // Clear reset
	enc28j60_wcr(cs, MACON3, MACON3_PADCFG0| // Enable padding,
		MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX); // Enable crc & frame len chk
	enc28j60_wcr16(cs, MAMXFL, ENC28J60_MAXFRAME);
	enc28j60_wcr(cs, MABBIPG, 0x15); // Set inter-frame gap
	enc28j60_wcr(cs, MAIPGL, 0x12);
	enc28j60_wcr(cs, MAIPGH, 0x0c);
	
    // enc28j60_wcr(MAADR5, macadr[0]); // Set MAC address
	// enc28j60_wcr(MAADR4, macadr[1]);
	// enc28j60_wcr(MAADR3, macadr[2]);
	// enc28j60_wcr(MAADR2, macadr[3]);
	// enc28j60_wcr(MAADR1, macadr[4]);
	// enc28j60_wcr(MAADR0, macadr[5]);

	// Setup PHY
	enc28j60_write_phy(cs, PHCON1, PHCON1_PDPXMD); // Force full-duplex mode
	enc28j60_write_phy(cs, PHCON2, PHCON2_HDLDIS); // Disable loopback
	enc28j60_write_phy(cs, PHLCON, PHLCON_LACFG2| // Configure LED ctrl
		PHLCON_LBCFG2|PHLCON_LBCFG1|PHLCON_LBCFG0|
		PHLCON_LFRQ0|PHLCON_STRCH);

	// Enable Rx packets
	enc28j60_bfs(cs, ECON1, ECON1_RXEN);
}

void enc28j60_send_packet(uint8_t cs, uint8_t *data, uint16_t len)
{
	while(enc28j60_rcr(cs, ECON1) & ECON1_TXRTS)
	{
		// TXRTS may not clear - ENC28J60 bug. We must reset
		// transmit logic in cause of Tx error
		if(enc28j60_rcr(cs, EIR) & EIR_TXERIF)
		{
			enc28j60_bfs(cs, ECON1, ECON1_TXRST);
			enc28j60_bfc(cs, ECON1, ECON1_TXRST);
		}
	}

	enc28j60_wcr16(cs, EWRPT, ENC28J60_TXSTART);
	enc28j60_write_buffer(cs, (uint8_t*)"\x00", 1);
	enc28j60_write_buffer(cs, data, len);

	enc28j60_wcr16(cs, ETXST, ENC28J60_TXSTART);
	enc28j60_wcr16(cs, ETXND, ENC28J60_TXSTART + len);

	enc28j60_bfs(cs, ECON1, ECON1_TXRTS); // Request packet send
}

// 0xC5 = 1100 0101
uint16_t enc28j60_recv_packet(uint8_t cs, uint8_t *buf, uint16_t buflen)
{
	uint16_t len = 0, status, temp;

    uint8_t num_packets = enc28j60_rcr(cs, EPKTCNT);
    if(num_packets)
	{
        debug_log("\r\nnum packets before read: %u\r\n", num_packets);
		enc28j60_wcr16(cs, ERDPT, enc28j60_rxrdpt[cs-1]);

		enc28j60_read_buffer(cs, (void*)&enc28j60_rxrdpt[cs-1], sizeof(enc28j60_rxrdpt[cs-1]));
		enc28j60_read_buffer(cs, (void*)&len, sizeof(len));
		enc28j60_read_buffer(cs, (void*)&status, sizeof(status));

		if(status & 0x80) //success
		{
			if(len > buflen) len = buflen;
			enc28j60_read_buffer(cs, buf, len);	
		}

		// Set Rx read pointer to next packet
		temp = (enc28j60_rxrdpt[cs-1] - 1) & ENC28J60_BUFEND;
		enc28j60_wcr16(cs, ERXRDPT, temp);

		// Decrement packet counter
		enc28j60_bfs(cs, ECON2, ECON2_PKTDEC);
	}

	return len;
}
