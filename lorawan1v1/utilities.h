
long random_at_most(long max);
void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size );
#ifdef ENABLE_VT100
void print_buf(const uint8_t* const buf, uint8_t len, const char* txt, uint8_t row);
#else
void print_buf(const uint8_t* const buf, uint8_t len, const char* txt);
#endif
bool ValueInRange( int8_t value, int8_t min, int8_t max );

#ifndef MIN
    #define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef MAX
    #define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif
