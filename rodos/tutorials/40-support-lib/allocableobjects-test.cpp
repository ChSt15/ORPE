
#include "rodos.h"
#include "allocableobjects.h"
#include "rodos-result.h"

struct TestObj {
    int a, b, c;
};

AllocableObjects<TestObj, 20> allocableTestObjs;


class TestAllocs : public StaticThread<> {
  public:
    SharedPtr<TestObj> ptr[50];
    void     run();
} testAllocs;

void TestAllocs::run() {
    allocableTestObjs.init();
    PRINTF("free object %d\n", allocableTestObjs.getNumOfFreeItems());

    PRINTF("allocate 25, but we have only 20\n");
    for(int i = 0; i < 25; i++) {
      PRINTF("    Counter %2d : ", i);
      Result<SharedPtr<TestObj>> allocResult = allocableTestObjs.alloc();
      if(allocResult.isOk()) {
        ptr[i] = allocResult;
        Result<uint32_t> x = allocableTestObjs.indexOf(ptr[i]);
        PRINTF(" index %d/%d, free = %d ptr = %x\n", x.isOk(), x.val, allocableTestObjs.getNumOfFreeItems(), (int)(intptr_t)ptr[i].getRawPointer());
      }
      else {
        PRINTF("  *** FAIL ** \n");
      }
    }

    PRINTF("copy the first 10 references \n");
    for(int i = 0; i < 10; i++) {
        ptr[25+i] = ptr[i]; 
        Result<uint32_t> x = allocableTestObjs.indexOf(ptr[i+15]);
        PRINTF("    Counter %2d :", i);
        if(x.isOk()) PRINTF("    index %d/%d, free = %d ptr = %x\n", x.isOk(), x.val, allocableTestObjs.getNumOfFreeItems(), (int)(intptr_t)ptr[i].getRawPointer());
        else PRINTF(" *** FAIL**\n");
    }

    PRINTF("free 40, only 35 are defined! the first 10 hast 2 copies, nothing shall be released!\n");
    for(int i = 0; i < 40; i++) {
        PRINTF("    Counter %2d :", i);
        ptr[i].clear();
        Result<uint32_t> x = allocableTestObjs.indexOf(ptr[i]);
        PRINTF("    ok = %d index %d, free = %d\n", x.isOk(), x.val, allocableTestObjs.getNumOfFreeItems());
    }
}
