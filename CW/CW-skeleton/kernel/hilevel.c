#include "hilevel.h"

// Initialise global variables, main progams and tos's
pcb_t pcb[ program_max ];
pcb_t* current = NULL;

extern void     main_P3();
extern uint32_t tos_P3;
extern void     main_P4();
extern uint32_t tos_P4;
extern void     main_P5();
extern uint32_t tos_P5;
extern void     main_console();
extern uint32_t tos_n;
extern uint32_t tos_console;


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

int next_pcb_index = 0;

//age all waiting programs by 1
while(i< program_max){
  if ((pcb[i].status == STATUS_TERMINATED) || (pcb[i].pri == 0) || pcb[i].status == STATUS_EXECUTING ){
    }
  else{
    if(i = 0){
      pcb[i].age = pcb[i].age + 3;
    }
    else{
    pcb[i].age = pcb[i].age + 1;
  }
}
  i++;
}


// find the next program pcb index
while(k< program_max){
   if((pcb[k].pri + pcb[k].age >= pcb[next_pcb_index].pri + pcb[next_pcb_index].age) && (pcb[k].status != STATUS_TERMINATED) && (pcb[k].pri != 0)){
     next_pcb_index = k;
   }
   k++;
   }


// if the next and the current are the same then reset age of current but thats it
if (current->pid == pcb[next_pcb_index].pid ){
  current->age = 0;
  return;}

// otherwise do a dispatch and update excecution status
else{


if(current->status != STATUS_TERMINATED){
  current->status = STATUS_READY;
}
pcb[next_pcb_index].status = STATUS_EXECUTING;
pcb[next_pcb_index].age = 0;


dispatch(ctx, current, &pcb[next_pcb_index] );}


return;


}


void hilevel_handler_rst(ctx_t* ctx) {

   int i =0;
   int j =1;


   // initialise console
    memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );
      pcb[ 0 ].pid      = 0;
      pcb[ 0 ].status   = STATUS_CREATED;
      pcb[ 0 ].ctx.cpsr = 0x50;
      pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
      pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_n  );
      pcb[0].age = 0;
      pcb[0].pri = 1;


      //initialise 1-18 blank pcb slots
      while(j<program_max){
      memset( &pcb[ j ], 0, sizeof( pcb_t ) );
      pcb[ j ].pid      = j;
      pcb[ j ].status   = STATUS_TERMINATED;
      pcb[ j ].ctx.cpsr = 0x50;
      pcb[ j ].ctx.pc   = ( uint32_t )( &main_console );
      pcb[ j ].ctx.sp   = (( uint32_t )(&tos_n)) - (j * 0x00001000 );
      pcb[j].age = 0;
      pcb[j].pri = 0;

      j++;
      }

  // dispatch to get the console running
  dispatch( ctx, NULL, &pcb[ 0 ] );


  //setup the timer
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


  case 0x04 : { // exit
    current->status = STATUS_TERMINATED;
    current->pri =0;
    current->age = 0;
    break;

  }

  case 0x03 : { // fork

      //Initislaise fork variables
      pcb_t* parent = current;
      pcb_t* child;


      // Find a free PCB slot for the child process
      int i =0;
      int z = -1;
      while(i< program_max){
        if (pcb[i].status == STATUS_TERMINATED || pcb[i].pri == 0){
        z = i;
        break;
        }
        i++;}

      // if no available pcb slot
      if (z == -1){
        ctx->gpr[0] = -1;
        break;
      }

      // set child to the free pcb
      child = &pcb[z];


      // Set age, priority and state
      memcpy( &child->ctx, ctx, sizeof( ctx_t ) );
      child->status = STATUS_READY;
      child->age = 0;
      child->pri = parent->pri;


      // Set child stack pointer and copy stack
      child->ctx.sp = (( uint32_t )(&tos_n)) -(0x00001000 * child->pid) - (((uint32_t)(&tos_n)) -(0x00001000 * parent->pid) - ctx->sp);
      memcpy((void*)(child->ctx.sp), (void*)(ctx->sp), (((uint32_t)(&tos_n)) -(0x00001000 * parent->pid) - ctx->sp));


      //return 0 for child, PID of child for parent
      child->ctx.gpr[0] = 0;
      ctx->gpr[0] = child->pid;

break;

  }


  case 0x05 :{ //exec

    // Set PC
    ctx->pc = ctx->gpr[0];

    break;
  }


  case 0x06:{ // kill
    pcb[ctx->gpr[0]].status = STATUS_TERMINATED;
    pcb[ctx->gpr[0]].pri =0;
    pcb[ctx->gpr[0]].age = 0;


    break;

  }




  default   : { // 0x?? => unknown/unsupported
      break;
    }

  return;
}
}
