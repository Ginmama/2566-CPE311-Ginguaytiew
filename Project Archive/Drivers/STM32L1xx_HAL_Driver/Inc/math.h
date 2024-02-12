#ifndef MATHS_H
#define MATHS_H
__attribute__((section(".eeprom"))) uint8_t eeprom_data = 0;
__attribute__((section(".my_flash_section"))) uint32_t flash_data = 0;
void save_to(uint8_t value) {eeprom_data = value;}
uint8_t load_from() {return eeprom_data;}
#endif
