// 这里保存了热力图的颜色表，格式为rgb565
#ifndef COLORMAP_H
#define COLORMAP_H

#include <Arduino.h>
#include "shared_val.h"

static uint16_t colormap[180];

uint16_t viridis[180] PROGMEM = {
0x400a, 0x400a, 0x400a, 0x400a, 0x400a, 0x400a, 0x402b, 0x402b, 0x404b, 0x404b, 
0x404b, 0x406b, 0x406b, 0x406c, 0x406c, 0x408c, 0x408c, 0x408c, 0x40ac, 0x40ac, 
0x40ad, 0x40ad, 0x48cd, 0x48cd, 0x48cd, 0x48cd, 0x48ed, 0x48ee, 0x48ee, 0x490e, 
0x490e, 0x490e, 0x490e, 0x490e, 0x490e, 0x412e, 0x412e, 0x412e, 0x414f, 0x414f, 
0x414f, 0x414f, 0x416f, 0x416f, 0x416f, 0x418f, 0x418f, 0x418f, 0x418f, 0x41af, 
0x41af, 0x41b0, 0x41b0, 0x41b0, 0x41d0, 0x41d0, 0x41d0, 0x41d0, 0x41f0, 0x41f0, 
0x41f0, 0x41f0, 0x4210, 0x4210, 0x4210, 0x4210, 0x4210, 0x4230, 0x4230, 0x3a31, 
0x3a31, 0x3a51, 0x3a51, 0x3a51, 0x3a51, 0x3a51, 0x3a71, 0x3a71, 0x3a71, 0x3a71, 
0x3a91, 0x3a91, 0x3a91, 0x3a91, 0x3a91, 0x3ab1, 0x3ab1, 0x3ab1, 0x3ab1, 0x3ab1, 
0x32d1, 0x32d1, 0x32d1, 0x32f1, 0x3311, 0x3311, 0x3331, 0x3331, 0x3351, 0x2b51, 
0x2b71, 0x2b71, 0x2b91, 0x2b91, 0x2bb1, 0x2bb1, 0x2bd1, 0x2bd1, 0x23f1, 0x23f1, 
0x23f1, 0x2411, 0x2431, 0x2431, 0x2451, 0x2451, 0x2451, 0x2471, 0x2491, 0x2491, 
0x1cb1, 0x1cb1, 0x1cd1, 0x1cd1, 0x1cd1, 0x1cf1, 0x1d10, 0x1d10, 0x1d10, 0x2530, 
0x2530, 0x2550, 0x2550, 0x2570, 0x2d6f, 0x2d8f, 0x2d8f, 0x2d8f, 0x35af, 0x35af, 
0x3dce, 0x3dce, 0x45ee, 0x45ee, 0x4e0d, 0x4e0d, 0x4e0d, 0x562c, 0x562c, 0x5e4c, 
0x664c, 0x666b, 0x6e6b, 0x766a, 0x766a, 0x768a, 0x7e89, 0x8689, 0x8ea8, 0x8ea8, 
0x96a8, 0x96c7, 0x9ec7, 0xa6c6, 0xa6c6, 0xaee6, 0xb6e5, 0xb6e5, 0xbee4, 0xc6e4, 
0xc703, 0xcf03, 0xd703, 0xd703, 0xdf03, 0xe703, 0xef23, 0xef23, 0xf723, 0xff24
};

uint16_t classic[180] PROGMEM = {
0x0002, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005, 0x0005, 0x0006, 0x0006, 0x0007, 
0x0007, 0x0008, 0x0008, 0x0009, 0x0009, 0x000a, 0x000a, 0x000b, 0x000b, 0x000c, 
0x000c, 0x000d, 0x000d, 0x000e, 0x000e, 0x000f, 0x000f, 0x0010, 0x0010, 0x0011, 
0x0011, 0x0011, 0x0811, 0x0810, 0x1010, 0x1010, 0x1810, 0x180f, 0x200f, 0x200f, 
0x280f, 0x280e, 0x300e, 0x300e, 0x380e, 0x380d, 0x400d, 0x400d, 0x480d, 0x480c, 
0x500c, 0x500c, 0x580c, 0x580b, 0x600b, 0x600b, 0x680b, 0x680a, 0x700a, 0x700a, 
0x780a, 0x7809, 0x8009, 0x8009, 0x8809, 0x8808, 0x9008, 0x9008, 0x9808, 0x9807, 
0xa007, 0xa007, 0xa807, 0xa806, 0xb006, 0xb006, 0xb806, 0xb805, 0xc005, 0xc005, 
0xc805, 0xc804, 0xd004, 0xd004, 0xd804, 0xd803, 0xe003, 0xe003, 0xe803, 0xe802, 
0xf801, 0xf801, 0xf821, 0xf821, 0xf841, 0xf841, 0xf861, 0xf861, 0xf881, 0xf880, 
0xf8a0, 0xf8a0, 0xf8c0, 0xf8c0, 0xf8e0, 0xf8e0, 0xf900, 0xf900, 0xf920, 0xf920, 
0xf940, 0xf940, 0xf960, 0xf960, 0xf980, 0xf980, 0xf9a0, 0xf9a0, 0xf9c0, 0xf9c0, 
0xf9e0, 0xfa00, 0xfa20, 0xfa60, 0xfa80, 0xfac0, 0xfae0, 0xfb20, 0xfb40, 0xfb80, 
0xfba0, 0xfbe0, 0xfc00, 0xfc20, 0xfc60, 0xfc80, 0xfcc0, 0xfce0, 0xfd20, 0xfd40, 
0xfd80, 0xfda0, 0xfde0, 0xfe00, 0xfe40, 0xfe60, 0xfe80, 0xfec0, 0xfee0, 0xff20, 
0xff40, 0xff41, 0xff62, 0xff63, 0xff64, 0xff65, 0xff66, 0xff67, 0xff88, 0xff89, 
0xff8a, 0xff8b, 0xff8c, 0xff8d, 0xffae, 0xffaf, 0xffb1, 0xffb2, 0xffb3, 0xffb4, 
0xffd5, 0xffd6, 0xffd7, 0xffd8, 0xffd9, 0xffda, 0xfffb, 0xfffc, 0xfffd, 0xfffe
};

uint16_t hot[180] PROGMEM = {
0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x1000, 0x1000, 0x1000, 0x1800, 
0x1800, 0x1800, 0x1800, 0x2000, 0x2000, 0x2000, 0x2000, 0x2800, 0x2800, 0x2800, 
0x3000, 0x3000, 0x3000, 0x3000, 0x3800, 0x3800, 0x3800, 0x4000, 0x4000, 0x4000, 
0x4000, 0x4800, 0x4800, 0x4800, 0x4800, 0x5000, 0x5000, 0x5000, 0x5000, 0x5000, 
0x5800, 0x5800, 0x5800, 0x5800, 0x6000, 0x6000, 0x6000, 0x6800, 0x6800, 0x6800, 
0x6800, 0x7000, 0x7000, 0x7000, 0x7800, 0x7800, 0x7800, 0x7800, 0x8000, 0x8000, 
0x8000, 0x8000, 0x8800, 0x8800, 0x8800, 0x9000, 0x9000, 0x9000, 0x9000, 0x9800, 
0x9800, 0x9800, 0x9800, 0x9800, 0xa000, 0xa000, 0xa000, 0xa000, 0xa800, 0xa800, 
0xa800, 0xb000, 0xb000, 0xb000, 0xb000, 0xb000, 0xb000, 0xb800, 0xb800, 0xb800, 
0xc000, 0xc000, 0xc800, 0xd000, 0xd000, 0xd800, 0xe000, 0xe800, 0xe800, 0xf000, 
0xf800, 0xf800, 0xf820, 0xf820, 0xf860, 0xf880, 0xf8c0, 0xf8e0, 0xf920, 0xf940, 
0xf960, 0xf9a0, 0xf9c0, 0xfa00, 0xfa20, 0xfa40, 0xfa60, 0xfaa0, 0xfac0, 0xfae0, 
0xfb20, 0xfb40, 0xfb80, 0xfb80, 0xfbc0, 0xfc00, 0xfc40, 0xfc40, 0xfc60, 0xfca0, 
0xfcc0, 0xfd00, 0xfd20, 0xfd60, 0xfd60, 0xfda0, 0xfdc0, 0xfde0, 0xfe20, 0xfe40, 
0xfe80, 0xfea0, 0xfee0, 0xff00, 0xff40, 0xff40, 0xff60, 0xffa0, 0xffc0, 0xffe0, 
0xffe1, 0xffe2, 0xffe3, 0xffe4, 0xffe5, 0xffe6, 0xffe7, 0xffe8, 0xffe9, 0xffea, 
0xffeb, 0xffec, 0xffee, 0xffee, 0xffef, 0xfff0, 0xfff1, 0xfff2, 0xfff3, 0xfff5, 
0xfff5, 0xfff7, 0xfff7, 0xfff8, 0xfff9, 0xfffa, 0xfffb, 0xfffc, 0xfffd, 0xfffe
};

uint16_t turbo[180] PROGMEM = {
0x3087, 0x3087, 0x30a8, 0x30a8, 0x30c9, 0x30c9, 0x30eb, 0x30eb, 0x310b, 0x392d, 
0x392d, 0x394e, 0x394e, 0x396f, 0x396f, 0x3990, 0x3990, 0x39b1, 0x39d2, 0x39d2, 
0x39f3, 0x39f3, 0x4214, 0x4214, 0x4235, 0x4235, 0x4256, 0x4277, 0x4277, 0x4297, 
0x4297, 0x42b8, 0x42b8, 0x42d9, 0x42d9, 0x42fa, 0x431a, 0x431a, 0x431b, 0x431b, 
0x435c, 0x435c, 0x435c, 0x435c, 0x439d, 0x439d, 0x439d, 0x43bd, 0x43bd, 0x43de, 
0x43de, 0x43fe, 0x43fe, 0x441f, 0x443f, 0x443f, 0x445f, 0x445f, 0x447f, 0x447f, 
0x447f, 0x447f, 0x449f, 0x44bf, 0x44bf, 0x3cdf, 0x3cdf, 0x3cff, 0x3cff, 0x3d1f, 
0x3d1f, 0x3d3f, 0x355f, 0x355f, 0x357e, 0x357e, 0x357e, 0x357e, 0x2dbe, 0x2dbe, 
0x2dbd, 0x2ddd, 0x2ddd, 0x25fd, 0x25fd, 0x261c, 0x261c, 0x263c, 0x263c, 0x263b, 
0x1e5b, 0x1e7a, 0x1e7a, 0x1eb9, 0x16d8, 0x16f7, 0x1f17, 0x1f36, 0x1f36, 0x2755, 
0x2774, 0x2774, 0x2f93, 0x3792, 0x3fb1, 0x3fb1, 0x4fd0, 0x57cf, 0x5fcd, 0x67ed, 
0x67ed, 0x77eb, 0x7feb, 0x87ea, 0x87e9, 0x97e8, 0x97e8, 0x9fe7, 0xa7e7, 0xa7c7, 
0xafc6, 0xb7c6, 0xbfa6, 0xbfa6, 0xc786, 0xcf66, 0xcf46, 0xd746, 0xd726, 0xdf06, 
0xdee6, 0xe6c7, 0xe6a7, 0xee87, 0xee67, 0xf647, 0xf627, 0xf607, 0xfde7, 0xfdc7, 
0xfd86, 0xfd86, 0xfd46, 0xfd06, 0xfcc5, 0xfcc5, 0xfca5, 0xfc64, 0xfc24, 0xfbe4, 
0xfbc3, 0xf383, 0xf363, 0xf322, 0xf302, 0xf2e2, 0xeaa1, 0xea81, 0xea41, 0xe241, 
0xe201, 0xd9e1, 0xd9c0, 0xd1a0, 0xd1a0, 0xd160, 0xc940, 0xc140, 0xc120, 0xb8e0, 
0xb8e0, 0xb0c0, 0xa8c0, 0xa8a0, 0xa080, 0x9880, 0x9060, 0x9040, 0x8040, 0x8020
};

uint16_t inferno[180] PROGMEM = {
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0001, 0x0002, 
0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0023, 0x0023, 0x0023, 0x0823, 0x0823, 
0x0824, 0x0824, 0x0824, 0x0824, 0x0845, 0x0845, 0x0845, 0x1045, 0x1045, 0x1046, 
0x1046, 0x1046, 0x1046, 0x1047, 0x1047, 0x1847, 0x1848, 0x1848, 0x1868, 0x1868, 
0x1868, 0x1868, 0x2069, 0x2069, 0x2049, 0x204a, 0x204a, 0x204a, 0x204a, 0x284a, 
0x284a, 0x284b, 0x284b, 0x304b, 0x304b, 0x304b, 0x304c, 0x304c, 0x304c, 0x304c, 
0x384c, 0x384c, 0x384c, 0x384c, 0x384c, 0x404c, 0x404c, 0x404d, 0x404d, 0x404d, 
0x404d, 0x484d, 0x484d, 0x484d, 0x486d, 0x486d, 0x486d, 0x486d, 0x506d, 0x506d, 
0x506d, 0x506d, 0x506d, 0x588d, 0x588d, 0x588d, 0x588d, 0x588d, 0x588d, 0x588d, 
0x60ad, 0x60ad, 0x60ad, 0x68ad, 0x68cd, 0x70cd, 0x70cd, 0x78ed, 0x78ed, 0x78ed, 
0x80ed, 0x810d, 0x890d, 0x890d, 0x892d, 0x912d, 0x912c, 0x992c, 0x994c, 0x994c, 
0xa14c, 0xa16c, 0xa16b, 0xa96b, 0xa98b, 0xb18b, 0xb18b, 0xb9aa, 0xb9aa, 0xb9aa, 
0xc1ca, 0xc1c9, 0xc1e9, 0xc9e9, 0xca09, 0xca28, 0xd228, 0xd248, 0xd248, 0xda67, 
0xda67, 0xda86, 0xdaa6, 0xe2c6, 0xe2c6, 0xe2e5, 0xe2e5, 0xeb05, 0xeb24, 0xeb44, 
0xeb64, 0xf363, 0xf3a3, 0xf3a3, 0xf3e2, 0xf3e2, 0xf402, 0xf421, 0xfc41, 0xfc61, 
0xfc81, 0xfca0, 0xfca0, 0xfce0, 0xfce0, 0xfd01, 0xfd41, 0xfd41, 0xfd82, 0xfd82, 
0xfda3, 0xfdc4, 0xfe05, 0xfe05, 0xfe25, 0xfe46, 0xf667, 0xf687, 0xf6a8, 0xf6e9, 
0xf6ea, 0xf72b, 0xf72c, 0xf74d, 0xf76e, 0xf78f, 0xf7b0, 0xf7b1, 0xf7d2, 0xfff3
};

uint16_t greys_r[180] PROGMEM = { // 白热
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0020, 0x0020, 0x0841, 
0x0841, 0x0841, 0x0841, 0x0841, 0x0841, 0x0861, 0x0861, 0x0861, 0x1082, 0x1082, 
0x1082, 0x1082, 0x1082, 0x1082, 0x10a2, 0x10a2, 0x10a2, 0x18c3, 0x18c3, 0x18c3, 
0x18c3, 0x18c3, 0x18c3, 0x18e3, 0x18e3, 0x18e3, 0x2104, 0x2104, 0x2104, 0x2104, 
0x2104, 0x2104, 0x2124, 0x2124, 0x2945, 0x2945, 0x2945, 0x2945, 0x2945, 0x2965, 
0x2965, 0x2965, 0x2965, 0x3186, 0x3186, 0x3186, 0x31a6, 0x31a6, 0x31a6, 0x31a6, 
0x39c7, 0x39c7, 0x39c7, 0x39e7, 0x39e7, 0x4208, 0x4208, 0x4208, 0x4208, 0x4228, 
0x4228, 0x4228, 0x4228, 0x4228, 0x4a49, 0x4a49, 0x4a49, 0x4a49, 0x4a69, 0x4a69, 
0x4a69, 0x528a, 0x528a, 0x528a, 0x528a, 0x52aa, 0x52aa, 0x52aa, 0x52aa, 0x52aa, 
0x5acb, 0x5acb, 0x5acb, 0x5aeb, 0x630c, 0x630c, 0x632c, 0x6b4d, 0x6b4d, 0x6b6d, 
0x6b6d, 0x6b6d, 0x738e, 0x738e, 0x73ae, 0x73ae, 0x7bcf, 0x7bef, 0x8410, 0x8410, 
0x8410, 0x8430, 0x8c51, 0x8c51, 0x8c71, 0x8c71, 0x9492, 0x94b2, 0x94b2, 0x94b2, 
0x9cd3, 0x9cf3, 0xa514, 0xa514, 0xa534, 0xa534, 0xad55, 0xad75, 0xad75, 0xb596, 
0xb596, 0xb5b6, 0xbdd7, 0xbdf7, 0xbdf7, 0xc618, 0xc618, 0xc618, 0xc638, 0xc638, 
0xce59, 0xce59, 0xce79, 0xce79, 0xd69a, 0xd69a, 0xd69a, 0xd6ba, 0xd6ba, 0xdedb, 
0xdedb, 0xdefb, 0xdefb, 0xdefb, 0xe71c, 0xe71c, 0xe71c, 0xe73c, 0xe73c, 0xe73c, 
0xef5d, 0xef5d, 0xef7d, 0xef7d, 0xef7d, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf7be, 
0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffff, 0xffff
};

uint16_t greys[180] PROGMEM = {  // 黑热
0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffdf, 
0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 0xffdf, 
0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 0xf7be, 
0xf7be, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 0xf79e, 
0xf79e, 0xf79e, 0xef7d, 0xef7d, 0xef7d, 0xef7d, 0xef7d, 0xef7d, 0xef7d, 0xef5d, 
0xef5d, 0xef5d, 0xef5d, 0xef5d, 0xef5d, 0xef5d, 0xe73c, 0xe73c, 0xe73c, 0xe73c, 
0xe73c, 0xe73c, 0xe73c, 0xe71c, 0xe71c, 0xe71c, 0xe71c, 0xe71c, 0xe71c, 0xe71c, 
0xe71c, 0xdefb, 0xdefb, 0xdefb, 0xdefb, 0xdefb, 0xdefb, 0xdefb, 0xdedb, 0xdedb, 
0xdedb, 0xdedb, 0xdedb, 0xd6ba, 0xd6ba, 0xd6ba, 0xd6ba, 0xd6ba, 0xd6ba, 0xd6ba, 
0xd69a, 0xd69a, 0xd69a, 0xce79, 0xce79, 0xce59, 0xce59, 0xc638, 0xc638, 0xc618, 
0xc618, 0xc618, 0xbdf7, 0xbdf7, 0xbdd7, 0xb5b6, 0xb596, 0xb596, 0xad75, 0xad75, 
0xad55, 0xa534, 0xa534, 0xa514, 0xa514, 0x9cf3, 0x9cd3, 0x94b2, 0x94b2, 0x94b2, 
0x9492, 0x8c71, 0x8c71, 0x8c51, 0x8c51, 0x8430, 0x8410, 0x8410, 0x8410, 0x7bef, 
0x7bcf, 0x73ae, 0x73ae, 0x738e, 0x738e, 0x6b6d, 0x6b6d, 0x6b6d, 0x6b4d, 0x6b4d, 
0x632c, 0x630c, 0x630c, 0x5aeb, 0x5acb, 0x5acb, 0x5acb, 0x52aa, 0x52aa, 0x528a, 
0x4a69, 0x4a49, 0x4a49, 0x4228, 0x4228, 0x4208, 0x39e7, 0x39c7, 0x31a6, 0x31a6, 
0x3186, 0x2965, 0x2945, 0x2945, 0x2945, 0x2104, 0x2104, 0x18e3, 0x18e3, 0x18c3, 
0x18c3, 0x10a2, 0x1082, 0x1082, 0x0861, 0x0861, 0x0841, 0x0841, 0x0020, 0x0000
};

// 根据索引从程序存储器中复制数据到静态数组
void load_colormap(uint8_t index) {
    cmap_loading_lock = true;
    if (index == COLORMAP_CLASSIC) {
        memcpy_P(colormap, classic, sizeof(colormap));
    }else if (index == COLORMAP_VIRIDIS) {
        memcpy_P(colormap, viridis, sizeof(colormap));
    }else if(index == COLORMAP_HOT) {
        memcpy_P(colormap, hot, sizeof(colormap));
    }else if(index == COLORMAP_TURBO) {
        memcpy_P(colormap, turbo, sizeof(colormap));
    }else if(index == COLORMAP_INFERNO) {
        memcpy_P(colormap, inferno, sizeof(colormap));
    }else if(index == COLORMAP_GRAYSR) {
        memcpy_P(colormap, greys_r, sizeof(colormap));
    }else if(index == COLORMAP_GRAYS) {
        memcpy_P(colormap, greys, sizeof(colormap));
    }else{
        memcpy_P(colormap, classic, sizeof(colormap));
    }
    cmap_loading_lock = false;
}

void next_cmap(){
    cmap_now_choose+=1;
    if (cmap_now_choose>=COLORMAP_GRAYS){
        cmap_now_choose = COLORMAP_CLASSIC;
    }
    load_colormap(cmap_now_choose);
}

void priv_cmap(){
    if (cmap_now_choose==COLORMAP_CLASSIC){
        cmap_now_choose = COLORMAP_GRAYS;
    }else{
        cmap_now_choose-=1;
    }
    load_colormap(cmap_now_choose);
}

#endif // COLORMAP_H