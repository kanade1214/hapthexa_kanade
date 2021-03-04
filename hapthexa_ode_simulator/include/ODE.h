#ifndef ODE_H
#define ODE_H

#include <ode/ode.h>
#include <functional>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#endif

class ODE {
private:
    static dWorldID world;
    static dSpaceID space;
    static dGeomID ground;
    static dJointGroupID contactgroup;
    static void *hapthexa;
    static dsFunctions fn;
    static std::function<void(int)> loop_;
public:
    ODE(std::function<void(int)> loop);
    void spin();
    ~ODE();
    dWorldID getWorld();
    dSpaceID getSpace();
    static void nearcallback(void *data, dGeomID o1, dGeomID o2);
    static void start();
    static void simloop(int pause);
    static void command(int cmd);
};

#endif