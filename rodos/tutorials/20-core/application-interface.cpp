
#include "rodos.h"


Application aa("aa", 100);
Application bb("bb", 200);
Application cc("cc", 300);


class Responder : public Application {
  public:
    Responder(const char* nm, const int32_t id) : Application(nm, id) {}
    int32_t request(void* requestMsg, size_t, void*, size_t) {
        int* i = (int*)requestMsg;
        return *i + 10;
    }
} responder("responder", 500);


class ExtraFunction : public Application {
    int cnt = 0;

  public:
    ExtraFunction(const char* nm, const int32_t id) : Application(nm, id) {}
    int getCnt() { return cnt++; }
} extraFunction("ExtraFunction", 700);

//__________________________________________________________
void MAIN() {
    Application* app;

    xprintf("find applications by id\n");
    for(int i = 0; i < 900; i+=100) {
        app = Application::findApplication(i);
        if(app != 0)
            xprintf("     app %d = %s\n", i, app->name);
        else
            xprintf("     app %d not found\n", i);
    }


    xprintf("Find by name and Request\n");
    app = Application::findApplication("xx");
    if(app == 0) xprintf("    as expected app xx not found\n");

    app = Application::findApplication("responder");
    if(app == 0) xprintf("     very bad, shall not be! crash comes\n");
    int par = 123;
    int i   = app->request(&par, sizeof(par), 0, 0);
    xprintf("     respond(%d) = %d\n", par, i);

    app = Application::findApplication("ExtraFunction");
    for(int i = 0; i < 10; i++) {
        int cnt = ((ExtraFunction*)app)->getCnt();
        xprintf("   cnt from extrafunction = %d\n", cnt);
    }
}
