#include "lorawan_board.h"


// Assumes 0 <= max <= RAND_MAX
// Returns in the closed interval [0, max]
long random_at_most(long max) {
    unsigned long
    // max <= RAND_MAX < ULONG_MAX, so this is okay.
    num_bins = (unsigned long) max + 1,
    num_rand = (unsigned long) RAND_MAX + 1,
    bin_size = num_rand / num_bins,
    defect   = num_rand % num_bins;

    long x;
    do {
        //x = random();
        x = rand();
    }
    // This is carefully written not to overflow
    while (num_rand - defect <= (unsigned long)x);

    // Truncated division is intentional
    return x/bin_size;
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void
#ifdef ENABLE_VT100
print_buf(const uint8_t* const buf, uint8_t len, const char* txt, uint8_t row)
{
    uint8_t i;
    vt.SetCursorPos(row, 1 );
    vt.printf("%s: ", txt);
    for (i = 0; i < len; i++)
        vt.printf("%02x ", buf[i]);
    vt.printf("\e[K");
#else
print_buf(const uint8_t* const buf, uint8_t len, const char* txt)
{
    uint8_t i;
    pc.printf("%s: ", txt);
    for (i = 0; i < len; i++)
        pc.printf("%02x ", buf[i]);
    pc.printf("\r\n");
#endif /* ENABLE_VT100 */
}

bool ValueInRange( int8_t value, int8_t min, int8_t max )
{
    if( ( value >= min ) && ( value <= max ) )
    {
        return true;
    }
    return false;
}

