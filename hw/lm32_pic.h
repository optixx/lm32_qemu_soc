#ifndef __LM32_PIC
#define __LM32_PIC
uint32_t lm32_pic_get_ip(struct lm32_pic *s);
uint32_t lm32_pic_get_im(struct lm32_pic *s);
void lm32_pic_set_ip(struct lm32_pic *s, uint32_t ip);
void lm32_pic_set_im(struct lm32_pic *s, uint32_t im);
#endif
