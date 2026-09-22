#include "../src/RainClosure.h"
#include <cassert>
#include <cstring>
#include <iostream>
using A = RainClosure::Action;
using I = RainClosure::Inputs;
I normal() { return {true,true,true,false,false,false,false,true,true}; }
RainClosure policy() { return {1000,30000,1500,10000}; }
void wet(RainClosure& r, I i) { assert(r.tick(0,i)==A::None); }
int main() {
    { auto r=policy(); auto i=normal(); wet(r,i);
      assert(r.tick(999,i)==A::None); assert(r.tick(1000,i)==A::Stop);
      assert(r.tick(2499,i)==A::None); assert(r.tick(2500,i)==A::Close);
      r.accepted(); i.closing=i.moving=true; i.stationary=false;
      assert(r.tick(2600,i)==A::None); i.closed=true; i.closing=i.moving=false;
      r.tick(2700,i); assert(!strcmp(r.phase,"closed")); assert(r.id==1);
      i.closed=false; assert(r.tick(100000,i)==A::None); }
    { auto r=policy(); auto i=normal(); i.moving=true; i.stationary=false; wet(r,i);
      assert(r.tick(1000,i)==A::Stop); i.moving=false;
      assert(r.tick(2500,i)==A::None); r.tick(11000,i); assert(!strcmp(r.block,"stop_timeout")); }
    { auto r=policy(); auto i=normal(); wet(r,i); r.tick(1000,i); r.cancel();
      assert(r.tick(5000,i)==A::None); assert(!strcmp(r.phase,"interrupted"));
      i.wet=false; r.tick(6000,i); r.tick(36000,i); i.wet=true; r.tick(37000,i);
      assert(r.tick(38000,i)==A::Stop); assert(r.id==2); }
    { auto r=policy(); auto i=normal(); i.automatic=false; wet(r,i);
      assert(r.tick(1000,i)==A::None); assert(!strcmp(r.block,"wrong_mode")); }
    { auto r=policy(); auto i=normal(); i.safe=false; i.moving=true; wet(r,i);
      assert(r.tick(1000,i)==A::Stop); i.moving=false;
      assert(r.tick(2000,i)==A::None); assert(!strcmp(r.block,"scope_not_safe"));
      i.safe=true; assert(r.tick(3000,i)==A::Stop); assert(r.tick(4500,i)==A::Close); }
    { auto r=policy(); auto i=normal(); i.ready=false; wet(r,i);
      assert(r.tick(1000,i)==A::None); assert(!strcmp(r.block,"not_ready"));
      i.ready=true; assert(r.tick(2000,i)==A::Stop); }
    { auto r=policy(); auto i=normal(); i.closing=i.moving=true; wet(r,i);
      assert(r.tick(1000,i)==A::Close); r.accepted(); i.fault=true;
      r.tick(2000,i); assert(!strcmp(r.phase,"error")); i.fault=false;
      assert(r.tick(3000,i)==A::None); }
    { auto r=policy(); auto i=normal(); wet(r,i); r.tick(1000,i); r.accepted();
      i.closing=i.moving=true; i.safe=false; assert(r.tick(2000,i)==A::Stop);
      i.safe=true; assert(r.tick(3000,i)==A::None); }
    { auto r=policy(); auto i=normal(); i.closed=true; wet(r,i); r.tick(1000,i);
      assert(!strcmp(r.phase,"closed")); }
    { auto r=policy(); auto i=normal(); wet(r,i); i.wet=false; r.tick(500,i);
      i.wet=true; r.tick(600,i); assert(r.tick(1599,i)==A::None);
      assert(r.tick(1600,i)==A::Stop); }
    { auto r=policy(); auto i=normal(); uint32_t t=UINT32_MAX-500;
      r.tick(t,i); assert(r.tick(t+1000,i)==A::Stop);
      assert(r.tick(t+2500,i)==A::Close); }
    { auto r=policy(); auto i=normal(); wet(r,i); r.tick(1000,i); r.fail("comm_error");
      assert(r.tick(100000,i)==A::None); assert(!strcmp(r.block,"comm_error")); }
    // Blocked while it rains, rain ends, AUTO returns much later: the stale
    // attempt must expire rather than close for weather that is long gone,
    // and it must release the open lockout while it is dry.
    { auto r=policy(); auto i=normal(); i.automatic=false; wet(r,i);
      assert(r.tick(1000,i)==A::None); assert(!strcmp(r.block,"wrong_mode"));
      assert(r.locked()); i.wet=false; r.tick(2000,i);
      assert(r.tick(31999,i)==A::None); assert(r.locked());
      assert(r.tick(32000,i)==A::None); assert(!strcmp(r.phase,"expired"));
      assert(!r.locked()); // open lockout gone once the attempt expired
      i.automatic=true; assert(r.tick(500000,i)==A::None);
      // A fresh episode still arms normally afterwards.
      i.wet=true; r.tick(501000,i); assert(r.tick(502000,i)==A::Stop); assert(r.id==2); }
    std::cout << "13 rain policy scenarios passed\n";
}
