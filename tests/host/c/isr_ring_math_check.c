#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define M 0x1FFu
static int32_t d9(uint32_t lo, uint32_t ref){uint32_t d=(lo-ref)&M;return (d>>8)?(int32_t)d-512:(int32_t)d;}
int main(void){
  srand(1); int32_t true_r=-70000, full=true_r; uint32_t ref=(uint32_t)full&M; long bad=0;
  for(int i=0;i<2000000;i++){ true_r += (rand()%511)-255; uint32_t lo=(uint32_t)true_r&M; full+=d9(lo,ref); ref=lo; if(full!=true_r)bad++; }
  uint32_t odo=0x7FFFFF00u; long bado=0;
  for(int i=0;i<1000000;i++){ uint32_t e=odo; uint32_t slot=e&0x7FFFFFFFu; odo+=rand()%1000; uint32_t now=odo; if(now-((now-slot)&0x7FFFFFFFu)!=e)bado++; }
  printf("raw mismatches %ld, edge mismatches %ld\n",bad,bado); return bad||bado;
}
