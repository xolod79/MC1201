//**************************************************************************
//*            Контроллер SD-карты
//**************************************************************************

module sdspi (


	// connections to io controller
   output [31:0]   sd_lba,
	output reg      sd_rd,
	output reg      sd_wr,
	input 	       sd_ack,

   input  [7:0]    sd_buff_addr,
   input  [15:0]   sd_buff_dout,
   output reg[15:0]  sd_buff_din,
   input           sd_buff_wr,

/*
   // порты SDкарты
   output reg      sdcard_cs, 
   output reg      sdcard_mosi, 
   output          sdcard_sclk, 
   input           sdcard_miso,
*/   
   output reg[3:0] sdcard_debug,       // отладочные сигналы

   input[26:0]     sdcard_addr,        // адрес сектора карты
   output reg      sdcard_idle,        // признак готовности контроллера
   input           sdcard_read_start,  // строб начала чтения
   input           sdcard_read_ack,    // флаг подтверждения окончания чтения
   output reg      sdcard_read_done,   // флаг окончагия чтения
   input           sdcard_write_start, // строб начала записи
   input           sdcard_write_ack,   // флаг подтверждения команды записи
   output reg      sdcard_write_done,  // флаг окончания записи
   output reg      sdcard_error,       // флаг ошибки
   input[7:0]      sdcard_xfer_addr,   // адрес в буфере чтния/записи
   output reg[15:0]sdcard_xfer_out,    // слово, читаемое из буфера чтения
   input           sdcard_xfer_write,  // строб записи буфера
   input[15:0]     sdcard_xfer_in,     // слово, записываемое в буфер записи
   input           controller_clk,     // тактирование буферных операций - тактовый сигнал процессорной шины
   input           mode,               // режим работы: 0 - ведомый, 1 - ведущий
   input           sdclk,              // тактовый сигнал SD-карты
   input           reset               // сброс контроллера
);

//  процесс записи:                                  процесс чтения
//   - заполняем буфер
//  host            sdspi                           host            sdspi
//-------------------------------                   -------------------------------
//  write_start=1                                   read_start=1
//                  write_done=1                                    read_done=1
//  write_ack=1                                     read_ack=1 
//                  write_done=0                                    read_done=0
//  write_ack=0                                     read_ack=0
//


   //****************************
    // буферная память
   //****************************
   reg[15:0] rsector[0:255]; // буфер чтения
   reg[15:0] wsector[0:255]; // буфер записи

   reg[9:0] counter; 
   reg[7:0] sectorindex; 
   reg[15:0] sd_word; 
   reg[3:0] idle_filter; 
   reg[3:0] read_start_filter; 
   reg[3:0] read_ack_filter; 
   reg[3:0] read_done_filter; 
   reg[3:0] write_start_filter; 
   reg[3:0] write_ack_filter; 
   reg[3:0] write_done_filter; 
   reg[3:0] card_error_filter; 
   
   reg idle; 
   reg read_start; 
   reg read_ack; 
   reg read_done; 
   reg write_start; 
   reg write_ack; 
   reg write_done; 
   reg card_error;
	
//*******************************************
//* Интерфейс к хост-модулю
//*******************************************
always @(posedge controller_clk) begin
         
         // операции с буферами
         sdcard_xfer_out <= rsector[sdcard_xfer_addr] ; // слово, читаемое из буфера чтения
         if (sdcard_xfer_write == 1'b1)  wsector[sdcard_xfer_addr] <= sdcard_xfer_in; // запись слова в буфер записи
         
         // сдвиговые регистры фильтров интерфейсных сигналов
         idle_filter <= {idle_filter[2:0], idle} ;  // фильтр сигнала готовности
         read_start_filter <= {read_start_filter[2:0], sdcard_read_start} ; // фильтр строба чтения
         read_ack_filter <= {read_ack_filter[2:0], sdcard_read_ack} ;       // фильтр подтверждения чтения
         read_done_filter <= {read_done_filter[2:0], read_done} ;           // фильтр сигнала окончания записи
         write_start_filter <= {write_start_filter[2:0], sdcard_write_start} ; // фильтр строба записи
         write_ack_filter <= {write_ack_filter[2:0], sdcard_write_ack} ; 
         write_done_filter <= {write_done_filter[2:0], write_done} ; 
         card_error_filter <= {card_error_filter[2:0], card_error} ; 
         
         // сброс контроллера
         if (reset == 1'b1)  begin
            sdcard_idle <= 1'b0 ; 
            read_start <= 1'b0 ; 
            read_ack <= 1'b0 ; 
            sdcard_read_done <= 1'b0 ; 
            write_start <= 1'b0 ; 
            write_ack <= 1'b0 ; 
            sdcard_write_done <= 1'b0 ; 
            sdcard_error <= 1'b0 ; 
         end
         
         // фильтры интерфейсных сигналов
         else  begin
            // сигнал готовности к обмену
            if (idle_filter == {4{1'b0}})     sdcard_idle <= 1'b0 ; 
            else if (idle_filter == {4{1'b1}}) sdcard_idle <= 1'b1 ;
            
            // сигнал начала чтения
            if (read_start_filter == {4{1'b0}}) read_start <= 1'b0 ; 
            else if (read_start_filter == {4{1'b1}})  read_start <= 1'b1 ; 

            // сигнал подтверждения чтения
            if (read_ack_filter == {4{1'b0}})      read_ack <= 1'b0 ; 
            else if (read_ack_filter == {4{1'b1}}) read_ack <= 1'b1 ; 

            // строб окончания чтения
            if (read_done_filter == {4{1'b0}}) sdcard_read_done <= 1'b0 ; 
            else if (read_done_filter == {4{1'b1}})  sdcard_read_done <= 1'b1 ; 

            // сигнал начала записи
            if (write_start_filter == {4{1'b0}})     write_start <= 1'b0 ; 
            else if (write_start_filter == {4{1'b1}})  write_start <= 1'b1 ; 

            // строб подтверждения записи
            if (write_ack_filter == {4{1'b0}})       write_ack <= 1'b0 ; 
            else if (write_ack_filter == {4{1'b1}})  write_ack <= 1'b1 ; 

            // строб окончания записи
            if (write_done_filter == {4{1'b0}}) sdcard_write_done <= 1'b0 ; 
            else if (write_done_filter == {4{1'b1}}) sdcard_write_done <= 1'b1 ; 

            // сигнал ошибки карты
            if (card_error_filter == {4{1'b0}}) sdcard_error <= 1'b0 ; 
            else if (card_error_filter == {4{1'b1}}) sdcard_error <= 1'b1 ; 
         end 
end 

//*******************************************	
// Интерфейс к SD-карте
//*******************************************

//assign sd_wr   = 1'b0;  // we never write ...

assign sd_lba = sdcard_addr[26:0];



always @(posedge sdclk) 
    begin
// ---------------- buffer read engine -----------------------
// the buffer itself. Can hold one sector
      sd_buff_din <= wsector[sd_buff_addr];
// ---------------- buffer write engine ----------------------
      if(sd_buff_wr) rsector[sd_buff_addr] <= sd_buff_dout;
    end 


//*******************************************	
//* Обработка обмена данными с картой по SPI
//*******************************************	
//always @(posedge controller_clk)  begin
always @(posedge sdclk)  begin
     // сброс
         if (reset == 1'b1) begin
               // заранее настраиваем регистры как после инициализации
               idle <= 1'b1 ;          
               read_done <= 1'b0 ;       // снимаем флаг окончания чтения
               write_done <= 1'b0 ;      // и записи
               card_error <= 1'b0 ;      // снимаем флаг ошибки
               sdcard_debug <= 4'b0011 ; 
         end
         else  begin
                  sdcard_debug[0] <= 1'b0 ;   
                  sdcard_debug[1] <= 1'b0;
                  sdcard_debug[3:2] <= 2'b00 ; 
                  idle <= 1'b1 ;   // флаг готовности к обмену 

									// запуск чтения
                           if (read_start == 1'b1 
                            & read_done == 1'b0 
                            & write_done == 1'b0 
                            & read_ack == 1'b0 
                            & write_ack == 1'b0)  begin
                               card_error <= 1'b0 ; 
                               idle <= 1'b0 ;    // снимаем флаг готовности
										 
   									 sd_rd <= 1'b1;  //!!!!!!!!!!

                               if  ( sd_ack == 1'b1 & sd_rd == 1'b1 ) begin
												sd_rd <= 1'b0;
												sd_wr <= 1'b0;
                                    read_done <= 1'b0 ;  // снимаем строб готовности данных
   		                      end

                               sdcard_debug[2] <= 1'b1 ; 
                           end 
                           
                           // запуск записи
                           if (write_start == 1'b1 
                            & read_done == 1'b0 
                            & write_done == 1'b0 
                            & read_ack == 1'b0 
                            & write_ack == 1'b0)  begin
                               card_error <= 1'b0 ; 
                               idle <= 1'b0 ; 

										 sd_wr <= 1'b1;  //!!!!!!!!!!
					
                               if  ( sd_ack == 1'b1 & sd_wr == 1'b1 ) begin
												sd_rd <= 1'b0;
												sd_wr <= 1'b0;
												write_done <= 1'b1 ;         // поднимаем строб окончания записи
									    end
										 
                               sdcard_debug[3] <= 1'b1 ; 
                           end 

                           // хост подтвержил окончание чтения
                           if (read_ack == 1'b1)  begin
                              read_done <= 1'b0 ;  // снимаем строб готовности данных
                              card_error <= 1'b0 ; 
                           end 

                           // хост подтвердил окончание записи
                           if (write_ack == 1'b1) begin
                              write_done <= 1'b0 ; 
                              card_error <= 1'b0 ; 
                           end 
                end

   end 

	
endmodule
