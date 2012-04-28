
// SPI
// by Tim O'Brien (t413.com), Sept. 16, 2010.


#ifndef SPI_H
#define SPI_H
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif

    
    void init_spi(void);
    unsigned char exchangeSPI(unsigned char byte);

    
#ifdef __cplusplus
}
#endif
#endif
