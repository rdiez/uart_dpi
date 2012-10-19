
/* Version 0.84 beta, October 2012.

  See the README file for information about this module.

  Copyright (c) 2011 R. Diez

  This source file may be used and distributed without
  restriction provided that this copyright statement is not
  removed from the file and that any derivative work contains
  the original copyright notice and the associated disclaimer.

  This source file is free software; you can redistribute it
  and/or modify it under the terms of the GNU Lesser General
  Public License version 3 as published by the Free Software Foundation.

  This source is distributed in the hope that it will be
  useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General
  Public License along with this source; if not, download it
  from http://www.gnu.org/licenses/
*/

// The Wishbone widths are defined to match the ones used in OpenRISC-based projects.
// If you change them, you may have to adjust the code underneath, especially signal wb_sel_i.
// See also UART_DPI_ADDR_WIDTH below.
// The bus granularity is one byte, use wb_sel_i to select which one of the 4 bytes returned
// holds the data requested.
`define UART_DPI_DATA_WIDTH 32

// Divisor Latch access bit in UART_DPI_REG_LCR.
`define UART_DPI_LCR_DL 7

// FIFO Control Register bits.
`define UART_DPI_FCR_FIFO_ENABLE_BIT     0
`define UART_DPI_FCR_BITS_OTHER_THAN_FIFO_ENABLE  7:`UART_DPI_FCR_FIFO_ENABLE_BIT+1
`define UART_DPI_FCR_CLEAR_RCVR_BIT      1
`define UART_DPI_FCR_CLEAR_XMIT_BIT      2
`define UART_DPI_FCR_DMA_MODE_BIT        3
`define UART_DPI_FCR_RESERVED__BITS     5:4
`define UART_DPI_FCR_TRIGGER_LEVEL_BITS 7:6

// FIFO trigger level values in the FCR register.
`define UART_DPI_FCR_TRIGGER_LEVEL_1   2'b00
`define UART_DPI_FCR_TRIGGER_LEVEL_4   2'b01
`define UART_DPI_FCR_TRIGGER_LEVEL_8   2'b10
`define UART_DPI_FCR_TRIGGER_LEVEL_14  2'b11

// Line Status Register bits.
`define UART_DPI_LSR_DR   0  // Data ready
`define UART_DPI_LSR_OE   1  // Overrun Error
`define UART_DPI_LSR_PE   2  // Parity Error
`define UART_DPI_LSR_FE   3  // Framing Error
`define UART_DPI_LSR_BI   4  // Break interrupt
`define UART_DPI_LSR_THRE 5  // Transmit FIFO is empty
`define UART_DPI_LSR_TEMT 6  // Transmitter Empty indicator
`define UART_DPI_LSR_EI   7  // Error in receive FIFO

 // Interrupt Enable register bits.
`define UART_DPI_IER_RDA  0 // Received Data available interrupt
`define UART_DPI_IER_THRE 1 // Transmitter Holding Register empty interrupt
`define UART_DPI_IER_RLS  2 // Receiver Line Status interrupt
`define UART_DPI_IER_MS   3 // Modem Status interrupt
`define UART_DPI_IER_MASK_REST 8'b11110000

// Interrupt Identification Register bits.
`define UART_DPI_IIR_IP             0  // Interrupt pending (bit value: 0=pending, 1=not pending)
`define UART_DPI_IIR_II            3:1 // Interrupt identification bits
`define UART_DPI_IIR_FIFO_ENABLED  7:6
`define UART_DPI_IIR_FIFO_ENABLED_YES  2'b11
`define UART_DPI_IIR_FIFO_ENABLED_NO   2'b00

// Interrupt identification values for the UART_DPI_IIR_II bits.
`define UART_DPI_IIR_RLS  3'b011 // Receiver Line Status
`define UART_DPI_IIR_RDA  3'b010 // Receiver Data Available
`define UART_DPI_IIR_TI   3'b110 // Timeout Indication
`define UART_DPI_IIR_THRE 3'b001 // Transmitter Holding Register empty
`define UART_DPI_IIR_MS   3'b000 // Modem Status


module uart_dpi
               #(
                 // For MinSoC, I have used a UART_DPI_ADDR_WIDTH of 5 here (the minimum width),
                 // which means that some address bits will be ignored and the UART registers will
                 // be accessible at multiple addresses (in blocks of 2^5 addresses).
                 // For OR10 I have used a UART_DPI_ADDR_WIDTH of 24.
                 // Keep in mind that the highest address byte [31:24] is usually fixed to APP_ADDR_UART
                 // for OpenRISC SoC designs.
                 // Note that I haven't tested the code with any other values.
                 UART_DPI_ADDR_WIDTH = 5,
                 tcp_port  = 5678,
                 port_name = "UART DPI",
                 welcome_message = "Welcome to the UART DPI simulated serial interface.\n\r",
                 character_timeout_clk_count = 100,  // See the README file on how to calculate this accurately, should you need it.

                 // Whether the TCP server listens on localhost / 127.0.0.1 only. Otherwise,
                 // it listens on all IP addresses, which means any computer
                 // in the network can connect to the UART DPI module.
                 parameter listen_on_local_addr_only = 1,

                 parameter receive_buffer_size  = (100 * 1024),
                 parameter transmit_buffer_size = (100 * 1024),

                 // Whether the C++ side prints informational messages to stdout.
                 // Error messages cannot be turned off and get printed to stderr.
                 parameter print_informational_messages = 1,

                 TRACE_DATA = 0
                )
                ( input  wire wb_clk_i,
                  input  wire wb_rst_i,  // There is no need to assert reset at the beginning.

                  input  wire [UART_DPI_ADDR_WIDTH-1:0]  wb_adr_i,
                  input  wire [`UART_DPI_DATA_WIDTH-1:0] wb_dat_i,
                  output wire [`UART_DPI_DATA_WIDTH-1:0] wb_dat_o,

                  input  wire       wb_we_i,
                  input  wire       wb_stb_i,
                  input  wire       wb_cyc_i,
                  output wire       wb_ack_o,
                  output wire       wb_err_o,  // Never set, any attempt to use an invalid address terminates the simulation.
                  input  wire [3:0] wb_sel_i,

                  output wire       int_o  // UART interrupt request
                );

   import "DPI-C" function int uart_dpi_create ( input integer  tcp_port,
                                                 input bit      listen_on_local_addr_only,
                                                 input int      transmit_buffer_size,
                                                 input int      receive_buffer_size,
                                                 input string   welcome_message,
                                                 input bit      print_informational_messages,
                                                 input string   informational_message_prefix,
                                                 output longint obj );

   import "DPI-C" function int uart_dpi_send    ( input longint obj, input  byte character );
   import "DPI-C" function int uart_dpi_receive ( input longint obj, output byte character );

   import "DPI-C" function int uart_dpi_tick ( input longint obj, output int received_byte_count );

   // It is not necessary to call uart_dpi_destroy(). However, calling it
   // will release all resources associated with the UART DPI instance, and that can help
   // identify resource or memory leaks in other parts of the software.
   import "DPI-C" function void uart_dpi_destroy ( input longint obj );


   // Register addresses, note that a few registers are actually mapped to the same address.
   localparam UART_DPI_REG_RBR   = 0; // Receiver buffer
   localparam UART_DPI_REG_THR   = 0; // Transmitter
   localparam UART_DPI_REG_IER   = 1; // Interrupt enable
   localparam UART_DPI_REG_IIR   = 2; // Interrupt identification
   localparam UART_DPI_REG_FCR   = 2; // FIFO control
   localparam UART_DPI_REG_LCR   = 3; // Line Control
   localparam UART_DPI_REG_MCR   = 4; // Modem control
   localparam UART_DPI_REG_LSR   = 5; // Line status
   localparam UART_DPI_REG_MSR   = 6; // Modem status
   localparam UART_DPI_REG_SCR   = 7; // Scratch register
   localparam UART_DPI_REG_DL_LS = 0; // Divisor latch, same address as UART_DPI_REG_THR.
   localparam UART_DPI_REG_DL_MS = 1; // Divisor latch, same address as UART_DPI_REG_IER.


   // ---- UART registers begin.
   reg [7:0] uart_reg_lcr;
   reg [7:0] uart_reg_fcr;
   reg [7:0] uart_reg_scr;
   reg [7:0] uart_reg_ier;
   reg [7:0] uart_reg_dl_ms;
   reg [7:0] uart_reg_dl_ls;
   //  ---- UART registers end.

   longint   obj;  // There can be several instances of this module, and each one has a diferent obj value,
                   // which is a pointer to a class instance on the C++ side.

   // The simulated UART is always ready to take new data to send, therefore
   // the THRE interrupt always triggers when enabled. This interrupt flag must be
   // cleared when the UART client reads the IIR, and is triggered again
   // as soon as the UART client sends the next data byte.
   bit       transmitter_holding_register_empty_interrupt_pending;
   int       last_rcvr_fifo_read_clk_counter;  // For the Character Timeout.


   `define UART_DPI_ERROR_PREFIX       { port_name, " error: " }
   `define UART_DPI_INFORMATION_PREFIX { port_name, ": " }
   `define UART_DPI_TRACE_PREFIX       { port_name, ": " }


   function int get_trigger_level;
      input [7:0] fcr;
      reg [7:0] _unused_ok = fcr;
      begin
         if ( fcr[ `UART_DPI_FCR_FIFO_ENABLE_BIT ] )
           case ( fcr[ `UART_DPI_FCR_TRIGGER_LEVEL_BITS ] )
             `UART_DPI_FCR_TRIGGER_LEVEL_1 : get_trigger_level = 1;
             `UART_DPI_FCR_TRIGGER_LEVEL_4 : get_trigger_level = 4;
             `UART_DPI_FCR_TRIGGER_LEVEL_8 : get_trigger_level = 8;
             `UART_DPI_FCR_TRIGGER_LEVEL_14: get_trigger_level = 14;
           endcase
         else
           get_trigger_level = 1;
      end
   endfunction

   // The Wishbone bus is 32-bit wide, so we need to extract the right 8 bits to write.
   function bit [7:0] get_data_to_write;
      input [3:0]                      sel;
      input [`UART_DPI_DATA_WIDTH-1:0] data;
      begin
         // TODO: is this comment at the right place here?
         // Alternatively, we could return all 32-bit data for 4 register addresses at a time
         // in wb_dat_o and let the master pick the 8 bits it needs.
	     case ( sel )
		   4'b0001 : get_data_to_write = data[7:0];
		   4'b0010 : get_data_to_write = data[15:8];
		   4'b0100 : get_data_to_write = data[23:16];
		   4'b1000 : get_data_to_write = data[`UART_DPI_DATA_WIDTH-1:24];
		   default :
             begin
                $display( "%sDefault case for wb_sel_i, write cycle.", `UART_DPI_ERROR_PREFIX );
                $finish;
                get_data_to_write = data[7:0];
             end
	     endcase;
      end
   endfunction


   // The Wishbone bus is 32-bit wide, so we need to place the 8 bits we return at the right location.
   function bit [`UART_DPI_DATA_WIDTH-1:0] get_data_to_return;
      input [3:0] sel;
      input [7:0] data;
      begin
		 case ( sel )
		   4'b0001: get_data_to_return = {24'b0, data};
		   4'b0010: get_data_to_return = {16'b0, data, 8'b0};
		   4'b0100: get_data_to_return = {8'b0, data, 16'b0};
		   4'b1000: get_data_to_return = {data, 24'b0};
 		   default:
             begin
                $display( "%sDefault case for wb_sel_i, read cycle.", `UART_DPI_ERROR_PREFIX );
                $finish;
                get_data_to_return = 0;
             end
         endcase;
      end
   endfunction;


   task automatic wishbone_write;
      input [7:0] data_to_write;
      begin
         case ( wb_adr_i )
           UART_DPI_REG_THR:
           // `UART_DPI_REG_RBR has the same value as UART_DPI_REG_THR and is only available when reading.
           // `UART_DPI_REG_DL_LS has the same value as UART_DPI_REG_THR.
             begin
                if ( uart_reg_lcr[ `UART_DPI_LCR_DL ] )
                  begin
                     // $display( "%sWriting to UART_DPI_REG_DL_LS data: 0x%02X", `UART_DPI_TRACE_PREFIX, data_to_write );

                     // Even though we ignore the baud rate, we remember the Divisor Latch register values,
                     // so that the client can read them back.
                     uart_reg_dl_ls <= data_to_write;
                  end
                else
                  begin
                     if ( TRACE_DATA )
                       $display( "%sWriting char data: %c (%d, 0x%02X)",
                                 `UART_DPI_TRACE_PREFIX,
                                 data_to_write >= 32 ? data_to_write : 8'd63, // show a question mark for weird codes
                                 data_to_write,
                                 data_to_write );

                     if ( 0 != uart_dpi_send( obj, data_to_write ) )
                       begin
                          $display( "%sError sending data.", `UART_DPI_ERROR_PREFIX );
                          $finish;
                       end;

                     // On the real UART, sending a byte clears the THRE interrupt, which will
                     // be enabled later when the character has been moved out of the
                     // Transmit Holding Register. However, this simulation acts like a very fast UART,
                     // so there is no time delay, the virtual UART is always ready to accept new data
                     // to send. Therefore, the THRE interrupt is immediately triggered if enabled.
                     // If this causes problems, we could add some delay like character_timeout_clk_count.
                     transmitter_holding_register_empty_interrupt_pending <= uart_reg_ier[ `UART_DPI_IER_THRE ];
                  end;
             end

           UART_DPI_REG_IER:
           // `UART_DPI_REG_DL_MS has the same value as UART_DPI_REG_IER.
             begin
                if ( uart_reg_lcr[ `UART_DPI_LCR_DL ] )
                  begin
                     // $display( "%sWriting to UART_DPI_REG_DL_MS data: 0x%02X", `UART_DPI_TRACE_PREFIX , data_to_write );

                     // Even though we ignore the baud rate, we remember the Divisor Latch register values,
                     // so that the client can read them back.
                     uart_reg_dl_ms <= data_to_write;
                  end
                else
                  begin
                     // $display( "%sWriting to UART_DPI_REG_IER data: 0x%02X", `UART_DPI_TRACE_PREFIX, data_to_write );

                     if ( 0 != ( data_to_write & `UART_DPI_IER_MASK_REST ) )
                       begin
                          $display( "%sThe client is setting reserved bits in the UART Interrupt Enable Register (IER).", `UART_DPI_ERROR_PREFIX );
                          $finish;
                       end;

                     // We don't support any modem functionality, so there will never be a modem status interrupt,
                     // even if we disable this error check here.
                     if ( 0 != ( data_to_write & (1 << `UART_DPI_IER_MS) ) )
                       begin
                          $display( "%sThe client is setting the Modem Status interrupt, which is not supported.", `UART_DPI_ERROR_PREFIX );
                          $finish;
                       end;

                     // The Receiver Line Status interrupt (bit ELSI) has to do with reception errors, but
                     // such errors are never reported by this virtual UART, so this interrupt
                     // will never trigger.

                     uart_reg_ier <= data_to_write;

                     // This simulated UART is always ready to accept new data to send. Therefore,
                     // if the client enables the THRE interrupt, it will trigger straight away.
                     // I am not certain how the real UART behaves if the client enables this interrupt
                     // and the outgoing FIFO is empty. Does the interrupt trigger immediately too,
                     // or does it wait for the first write to the THR?
                     transmitter_holding_register_empty_interrupt_pending <= data_to_write[ `UART_DPI_IER_THRE ];
                  end;
             end


           UART_DPI_REG_FCR:
           // `UART_DPI_REG_IIR has the same value as `UART_DPI_REG_FCR and is only available when reading.
             begin
                // $display( "%sWriting to UART_DPI_REG_FCR data: 0x%02X", `UART_DPI_TRACE_PREFIX, data_to_write );

                if ( data_to_write[ `UART_DPI_FCR_FIFO_ENABLE_BIT ] )
                  begin
                     // Some of the bits are self-clearing, but that is not implemented,
                     // as the FCR cannot be read back by the client.

                     if ( data_to_write[ `UART_DPI_FCR_CLEAR_RCVR_BIT ] )
                       begin
                          // Clearing the receive FIFO is not supported,
                          // the request is just ignored.
                       end;

                     if ( data_to_write[ `UART_DPI_FCR_CLEAR_XMIT_BIT ] )
                       begin
                          // Clearing the transmit FIFO is not supported, as all bytes
                          // land in the TCP transmit queue straight away,
                          // so it's as if they had been sent without transmission delay.
                          // Therefore this request is just ignored.
                       end;

                     if ( 0 != data_to_write[ `UART_DPI_FCR_RESERVED__BITS ] )
                       begin
                          $display( "%sThe client is setting the reserved bits in the UART FIFO Control Register (FCR), which is probably an error.", `UART_DPI_ERROR_PREFIX );
                          $finish;
                       end;
                  end
                else
                  begin
                     // If the FIFO ENABLE bit is not set, then all other bits should be ignored.
                     // But the client should not actually do that. At least the trigger level
                     // will cause unexpected behaviour if not set to 1 when the receive FIFO is disabled.
                     if ( 0 != data_to_write[ `UART_DPI_FCR_BITS_OTHER_THAN_FIFO_ENABLE ] )
                       begin
                          $display( "%sThe client is not writing to the UART FIFO Control Register (FCR) a coherent value, which is probably an error.", `UART_DPI_ERROR_PREFIX );
                          $finish;
                       end;
                  end;

                uart_reg_fcr <= data_to_write;
             end

           UART_DPI_REG_LCR:
             begin
                // Note that only the Divisor Latch Access Bit (DLAB) is considered,
                // all other line settings (like parity or number of stop bits) are ignored.
                // We always send and receive full bytes (8 bits), although we could filter
                // some bits depending on the Character Length setting. I'm not sure
                // it's worth the effort, though, as 8N1 is the standard setting for
                // text console applications.
                uart_reg_lcr <= data_to_write;
             end

           UART_DPI_REG_MCR:
             begin
                $display( "%sWriting to the UART Modem Control Register (MCR) is not supported.", `UART_DPI_ERROR_PREFIX );
                $finish;
             end

           UART_DPI_REG_MSR:
             begin
                $display( "%sWriting to the UART Modem Status Register (MSR) is not supported.", `UART_DPI_ERROR_PREFIX );
                $finish;
             end

           UART_DPI_REG_LSR:
             begin
                $display( "%sThe client is trying to write to the UART Line Status Register (LSR), which is intended for factory testing only and discouraged by the UART documentation.", `UART_DPI_ERROR_PREFIX );
                $finish;
             end

           UART_DPI_REG_SCR:
             uart_reg_scr <= data_to_write;

           default:
             begin
                $display( "%sDefault case for wb_adr_i=0x%02X, write cycle.", `UART_DPI_ERROR_PREFIX, wb_adr_i );
                $finish;
             end
         endcase;
      end
   endtask


   task automatic wishbone_read;
      input  int received_byte_count;
      input  bit receive_data_available_interrupt_pending;
      input  bit character_timeout_interrupt_pending;
      output bit [7:0] data_to_return;
      begin
         case ( wb_adr_i )
           UART_DPI_REG_RBR:
           // UART_DPI_REG_THR has the same value as UART_DPI_REG_RBR and is only available when writing.
           // UART_DPI_REG_DL_LS has the same value as UART_DPI_REG_RBR.
             if ( uart_reg_lcr[ `UART_DPI_LCR_DL ] )
               begin
                  data_to_return = uart_reg_dl_ls;
               end
             else
               begin
                  if ( received_byte_count == 0 )
                    begin
                       // Reading when the FIFO is empty is most probably an error in the client software,
                       // at least for the purposes of this simulated UART.
                       // I'm not sure what the real UART will return in this case, it will probably be
                       // the last byte received. If you need that kind of behaviour,
                       // uncomment the $finish below, and modify the code so that the last byte
                       // is remembered and returned at this point.
                       $display( "%sThe client is trying to receive a character, but the FIFO is empty.", `UART_DPI_ERROR_PREFIX );
                       $finish;
                       data_to_return = 0;  // Prevents C++ compilation warning under Verilator.
                    end
                  else if ( 0 != uart_dpi_receive( obj, data_to_return ) )
                    begin
                       $display( "%sError receiving data.", `UART_DPI_ERROR_PREFIX );
                       $finish;
                    end;

                  if ( TRACE_DATA )
                    $display( "%sReceived char data: %c (%d, 0x%02X)",
                              `UART_DPI_TRACE_PREFIX,
                              data_to_return >= 32 ? data_to_return : 8'd63 /* question mark */,
                              data_to_return,
                              data_to_return );

                  last_rcvr_fifo_read_clk_counter <= character_timeout_clk_count;
               end

           UART_DPI_REG_IER:
           // UART_DPI_REG_DL_MS has the same value as UART_DPI_REG_IER.
             begin
                if ( uart_reg_lcr[ `UART_DPI_LCR_DL ] )
                  begin
                     data_to_return = uart_reg_dl_ms;
                  end
                else
                  begin
                     data_to_return = uart_reg_ier;
                  end;
             end

           UART_DPI_REG_IIR:
           // `UART_DPI_REG_FCR has the same value as `UART_DPI_REG_IIR and is only available when writing.
             begin
                // About interrupts:
                // - There can never be a "Receiver Line Status" interrupt, as the errors it reports
                //   can never happen here, see the UART_DPI_REG_LSR logic below.
                // - There can never be a "MODEM Status" interrupt, as the MODEM logic is not supported.

                data_to_return = 0;  // A value of 0 in bit UART_DPI_IIR_IP means "interrupt is pending".

                // This sequence of the following 'if' statements determines the interrupt identification priority.
                if ( receive_data_available_interrupt_pending )
                  begin
                     data_to_return[ `UART_DPI_IIR_II ] = `UART_DPI_IIR_RDA;
                  end
                else if ( character_timeout_interrupt_pending )
                  begin
                     data_to_return[ `UART_DPI_IIR_II ] = `UART_DPI_IIR_TI;
                  end
                else if ( transmitter_holding_register_empty_interrupt_pending )
                  begin
                     data_to_return[ `UART_DPI_IIR_II ] = `UART_DPI_IIR_THRE;
                  end
                else
                  begin
                     data_to_return[ `UART_DPI_IIR_IP ] = 1;  // No interrupt pending.
                  end;

                data_to_return[ `UART_DPI_IIR_FIFO_ENABLED ] = uart_reg_fcr[`UART_DPI_FCR_FIFO_ENABLE_BIT]
                                                                  ? `UART_DPI_IIR_FIFO_ENABLED_YES
                                                                  : `UART_DPI_IIR_FIFO_ENABLED_NO;

                // Reading the IIR switches the THRE interrupt off.
                transmitter_holding_register_empty_interrupt_pending <= 0;
             end

           UART_DPI_REG_LCR:
             data_to_return = uart_reg_lcr;

           UART_DPI_REG_MCR:
             begin
                $display( "%sReading the UART Modem Control Register (MCR) is not supported.", `UART_DPI_ERROR_PREFIX );
                $finish;
                // In case you comment out the error above:
                data_to_return = 0;
             end

           UART_DPI_REG_MSR:
             begin
                $display( "%sReading the UART Modem Status Register (MSR) is not supported.", `UART_DPI_ERROR_PREFIX );
                $finish;
                // In case you comment out the error above:
                data_to_return = 0;
             end

           UART_DPI_REG_LSR:
             begin
                // Always say we can accept a new byte. This simulated UART
                // appears unnaturally fast to the user.
                // If this causes problems, we could add some delay like character_timeout_clk_count.
                data_to_return = (1 << `UART_DPI_LSR_THRE) |
                                 (1 << `UART_DPI_LSR_TEMT) ;

                // There are never bit-level errors like these:
                //   UART_DPI_LSR_PE   Parity Error
                //   UART_DPI_LSR_FE   Framing Error
                //   UART_DPI_LSR_BI   Break interrupt
                // Therefore, there can also never be a UART_DPI_LSR_EI (Error indicator).

                // There can never be a UART_DPI_LSR_OE (Overrun Error),
                // because the C++ side only reads from the socket
                // if there is room left in the receive buffer. If the TCP client
                // gives up, you will not notice at this side, but the client
                // will hopefully print an error message on its side.

                if ( received_byte_count > 0 )
                  data_to_return |= (1 << `UART_DPI_LSR_DR);
             end

           UART_DPI_REG_SCR:
             data_to_return = uart_reg_scr;

           default:
             begin
                $display( "%sDefault case for wb_adr_i=0x%02X, read cycle.", `UART_DPI_ERROR_PREFIX, wb_adr_i );
                $finish;
                // In case you comment out the error above:
                data_to_return = 0;
             end
         endcase;
      end
   endtask


   task automatic initial_reset;
      begin
         uart_reg_lcr   = 0;
         uart_reg_fcr   = 0;
         uart_reg_scr   = 0;
         uart_reg_ier   = 0;
         uart_reg_dl_ms = 0;
         uart_reg_dl_ls = 0;

         wb_dat_o       = 0;
         wb_ack_o       = 0;
         wb_err_o       = 0;
         int_o          = 0;

         transmitter_holding_register_empty_interrupt_pending = 0;
         last_rcvr_fifo_read_clk_counter = 0;
      end
   endtask


   always @(posedge wb_clk_i)
   begin
      int received_byte_count;

      // The TCP socket continues to be served even during reset.
      if ( 0 != uart_dpi_tick( obj, received_byte_count ) )
        begin
           $display( "%sError calling uart_dpi_tick().", `UART_DPI_ERROR_PREFIX );
           $finish;
        end;

      if ( wb_rst_i )
        begin
           // NOTE: If you modify the reset logic, please update the initial_reset task too.

           // According to the 16550 documentation, the chip's master reset does not clear
           // the Receiver Buffer, the Transmitter Holding and the Divisor Latches.
           uart_reg_lcr   <= 0;
           uart_reg_fcr   <= 0;
           uart_reg_scr   <= 0;
           uart_reg_ier   <= 0;
           uart_reg_dl_ms <= 0;
           uart_reg_dl_ls <= 0;

           wb_dat_o       <= 0;
           wb_ack_o       <= 0;
           wb_err_o       <= 0;
           int_o          <= 0;

           transmitter_holding_register_empty_interrupt_pending <= 0;
           last_rcvr_fifo_read_clk_counter <= 0;
	    end
      else
        begin
           bit receive_data_available_interrupt_pending;
           bit character_timeout_interrupt_pending;
           bit is_interrupt_pending;

           if ( last_rcvr_fifo_read_clk_counter != 0 )
             last_rcvr_fifo_read_clk_counter <= last_rcvr_fifo_read_clk_counter - 1;


           // Calculate the interrupt request signal, which is independent of the Wishbone bus.

           receive_data_available_interrupt_pending = uart_reg_ier[ `UART_DPI_IER_RDA ] &&
                                                      ( received_byte_count >= get_trigger_level( uart_reg_fcr ) );

           character_timeout_interrupt_pending = uart_reg_ier[ `UART_DPI_IER_RDA ] &&
                                                 ( received_byte_count >= 1 ) &&
                                                 ( last_rcvr_fifo_read_clk_counter == 0 ) &&
                                                 uart_reg_fcr[`UART_DPI_FCR_FIFO_ENABLE_BIT];

           is_interrupt_pending = receive_data_available_interrupt_pending |
                                  character_timeout_interrupt_pending |
                                  transmitter_holding_register_empty_interrupt_pending;

           int_o <= is_interrupt_pending;


           // Default values for the other output signals.
           wb_dat_o <= 0;
           wb_ack_o <= 0;


           if ( wb_cyc_i &&
                wb_stb_i &&
                !wb_ack_o  // If we answered in the last cycle, finish the transaction in this one by clearing wb_ack_o.
             )
             begin
                wb_ack_o <= 1;  // We can always answer straight away, without delays.

                if ( wb_we_i )
                  begin
                     bit [7:0] data_to_write;

                     data_to_write = get_data_to_write( wb_sel_i, wb_dat_i );

                     wishbone_write( data_to_write );

                     // $display( "%sWriting cycle, addr: 0x%X, data: %c",
                     //           `UART_DPI_TRACE_PREFIX,
                     //           wb_adr_i,
                     //           data_to_write );
                  end
                else
                  begin
                     bit [7:0] data_to_return;

                     wishbone_read( received_byte_count,
                                    receive_data_available_interrupt_pending,
                                    character_timeout_interrupt_pending,
                                    data_to_return );

                     // $display( "%sReading cycle, addr: 0x%08X, value returned: 0x%02X",
                     //           `UART_DPI_TRACE_PREFIX,
                     //           wb_adr_i,
                     //           data_to_return );

                     wb_dat_o <= get_data_to_return( wb_sel_i, data_to_return );
                  end;
             end;
        end;
   end;

   initial
     begin
        obj = 0;

        if ( 0 != uart_dpi_create( tcp_port,
                                   listen_on_local_addr_only,
                                   transmit_buffer_size,
                                   receive_buffer_size,
                                   welcome_message,
                                   print_informational_messages,
                                   `UART_DPI_INFORMATION_PREFIX,
                                   obj ) )
          begin
             $display( "%sError creating the object instance.", `UART_DPI_ERROR_PREFIX );
             $finish;
          end;

        initial_reset;
     end

   final
     begin
        // This is optional, but can help find resource or memory leaks in other parts of the software.
        uart_dpi_destroy( obj );
     end

endmodule
