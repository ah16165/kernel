/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"


pcb_t pcb[ program_max ];
pcb_t* current = NULL;
int programme_count;

extern void     main_P3();
extern uint32_t tos_P3;
extern void     main_P4();
extern uint32_t tos_P4;
extern void     main_P5();
extern uint32_t tos_P5;
extern void     main_console();
extern uint32_t tos_general;
extern uint32_t tos_console;
uint32_t general = (uint32_t) (&tos_general);

int find_current_pcb(){
  int j=0;
  while(j< program_max){
     if(pcb[j].pid == current->pid){
       return j;
     }
     j++;

     }


}

int free_pcb_slot(){
  int i =0;
  while(i< program_max){
    if (pcb[i].status == STATUS_TERMINATED){
      return i;
    }
    i++;
  }
}

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

                               // update   executing index   to P_{next}
                               current = next;
  return;
}

void schedule( ctx_t* ctx ) {
int i = 0;
int k = 0;
int current_pcb_index;
int next_pcb_index = 0;

//age all programs by 1
while(i< program_max){
  if ((pcb[i].status != STATUS_TERMINATED) ){
    pcb[i].age = pcb[i].age + 1;}
  i++;

}

// find the current program pcb index

current_pcb_index = find_current_pcb();

// find the next program pcb index
while(k< program_max){
   if((pcb[k].pri + pcb[k].age >= pcb[next_pcb_index].pri + pcb[next_pcb_index].age) && (pcb[k].status != STATUS_TERMINATED)){
     next_pcb_index = k;
   }
   k++;
   }
   // char check = '0' + pcb[current_pcb_index].pid;
   // PL011_putc( UART0, check,      true );






// if the next and the current are the same then do nothing
if (current == &pcb[next_pcb_index] ){
  PL011_putc( UART0, 'x',      true );
  pcb[next_pcb_index].age = 0;
  return;}

// otherwise do a dispatch and update excecution status'
else{

PL011_putc( UART0, 'y',      true );
//set the next pcb age to 0

dispatch(ctx, &pcb[current_pcb_index], &pcb[next_pcb_index] );}


// TIMER0->Timer1IntClr = 0x01;

pcb[current_pcb_index].status = STATUS_READY;
pcb[next_pcb_index].status = STATUS_EXECUTING;
pcb[next_pcb_index].age = 0;
current = &pcb[next_pcb_index];

return;


}






void hilevel_handler_rst(ctx_t* ctx) {
  /* Configure the mechanism for interrupt handling by
   *
   * - configuring timer st. it raises a (periodic) interrupt for each
   *   timer tick,
   * - configuring GIC st. the selected interrupts are forwarded to the
   *   processor via the IRQ interrupt signal, then
   * - enabling IRQ interrupts.
   */
   int i =0;
   int j =1;
   programme_count = 0;

   // initialise console
    memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );
      pcb[ 0 ].pid      = 0;
      pcb[ 0 ].status   = STATUS_CREATED;
      pcb[ 0 ].ctx.cpsr = 0x50;
      pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
      pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_general  );
      pcb[0].age = 0;
      pcb[0].pri = 1;

      // programme_count++;

      //initialise 1-32 blank pcb slots
      while(j<program_max){
      memset( &pcb[ j ], 0, sizeof( pcb_t ) );
      pcb[ j ].pid      = j;
      pcb[ j ].status   = STATUS_TERMINATED;
      pcb[ j ].ctx.cpsr = 0x50;
      pcb[ j ].ctx.pc   = ( uint32_t )( &main_console );
      pcb[ j ].ctx.sp   = ( uint32_t )( &tos_general - (0x00001000 * j));
      pcb[j].age = 0;
      pcb[j].pri = 0;

      j++;
      }

  // current = &pcb[ 0 ];
  dispatch( ctx, NULL, &pcb[ 0 ] );


  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();

  // // make all pcb = 0
  // while(i<program_max){
  //   memset( &pcb[ i ], 0, sizeof( pcb_t ) );
  //   pcb[i].pri = 0;
  // }



  return;
}

void hilevel_handler_irq(ctx_t* ctx) {


// read  the interrupt identifier so we know the source.
  uint32_t id = GICC0->IAR;

//handle the interrupt, then clear (or reset) the source.
  if( id == GIC_SOURCE_TIMER0 ) {

  schedule(ctx);

  TIMER0->Timer1IntClr = 0x01;
}

// write the interrupt identifier to signal we're done.
GICC0->EOIR = id;


  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

   switch( id ) {
     case 0x00 : { // 0x00 => yield()
       schedule( ctx );

       break;
     }

     case 0x01 : { // 0x01 => write( fd, x, n )
    int   fd = ( int   )( ctx->gpr[ 0 ] );
    char*  x = ( char* )( ctx->gpr[ 1 ] );
    int    n = ( int   )( ctx->gpr[ 2 ] );

    for( int i = 0; i < n; i++ ) {
      PL011_putc( UART0, *x++, true );
    }

    ctx->gpr[ 0 ] = n;

    break;
  }


  case 0x02: { //read
    int   fd = ( int   )( ctx->gpr[ 0 ] );
    char*  x = ( char* )( ctx->gpr[ 1 ] );
    int    n = ( int   )( ctx->gpr[ 2 ] );

    for( int i = 0; i < n; i++ ) {
      x[i] = PL011_getc( UART0, true );

  }
  ctx->gpr[ 0 ] = n;
  break;
}

  case 0x04 : { // exit
    int current_pcb_index = find_current_pcb();
    pcb[current_pcb_index].status = STATUS_TERMINATED;
    pcb[current_pcb_index].pri = 0;
    pcb[current_pcb_index].age = 0;
    schedule(ctx);



    break;

  }

  case 0x03 : { // fork


      int child_pcb = free_pcb_slot();
      int parent_pcb = find_current_pcb();
      pcb_t* parent = current;
      pcb_t child = pcb[child_pcb];
      // char check = '0' + child_pcb;
      //
      // PL011_putc( UART0, check,      true );


      //Set child PCB and pid
      // memset( &pcb[ child_pcb ], 0, sizeof( pcb_t ) );
      child.pid  = child_pcb;


      // Set age, priority and state
      memcpy( &child.ctx, ctx, sizeof( ctx_t ) );
      child.status = STATUS_READY;
      child.age = 0;
      child.pri = 3;

      child.ctx.pc = ctx->pc;


      // char check = '0' + child.pri;

      // PL011_putc( UART0, check,      true );

      // Set Sp and status,

      uint32_t x = general -(0x00001000 * parent->pid) - ctx->sp;


      child.ctx.sp = ( uint32_t )(general -(0x00001000 * parent->pid) - x);
      memcpy((void*)(child.ctx.sp), (void*)(ctx->sp),x);


      // programme_count = programme_count + 1;

      //return 0 for child, PID of child for parent
      child.ctx.gpr[0] = 0;
      ctx->gpr[0] = child.pid;


      PL011_putc( UART0, 'f',      true );


break;

  }



  case 0x05 :{ //exec

    // ctx->sp = tos_general;
    // ctx->cpsr = 0x50;

    // set pc to start of new function
    ctx->pc = ctx->gpr[0];
    PL011_putc( UART0, 'e',      true );

    break;



  }

  default   : { // 0x?? => unknown/unsupported
      break;
    }



  return;
}
}
