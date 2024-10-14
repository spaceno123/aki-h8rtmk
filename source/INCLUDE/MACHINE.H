/*
	machine.h
*/

void set_ccr( unsigned char );
unsigned char get_ccr( void );
void and_ccr( unsigned char );
void or_ccr( unsigned char );
void xor_ccr( unsigned char );

unsigned char get_and_ccr( unsigned char );	/* return before ccr data */
unsigned char get_or_ccr( unsigned char );	/* return before ccr data */
unsigned char get_xor_ccr( unsigned char );	/* return before ccr data */
