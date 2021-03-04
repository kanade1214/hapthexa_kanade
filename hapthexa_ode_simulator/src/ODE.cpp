
#include "ODE.h"

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif

//ODE関連
dWorldID ODE::world;
dSpaceID ODE::space;
dGeomID ODE::ground;
dJointGroupID ODE::contactgroup;
void *ODE::hapthexa;
dsFunctions ODE::fn;
std::function<void(int)> ODE::loop_;

void ODE::nearcallback(void *data, dGeomID o1, dGeomID o2)
{
    (void) data;
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;
    // if ((o1 != ground) && (o2 != ground)) return;

    static const int N = 20;
    dContact contact[N];
    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n > 0)
    {
        for (int i = 0; i < n; i++)
        {
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
            contact[i].surface.mu = dInfinity; //2.0;
            contact[i].surface.soft_erp = 0.9;
            contact[i].surface.soft_cfm = 1e-5;
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c, b1, b2);
        }
    }
}

void ODE::start()
{
    float xyz[3] = {2.0f, -1.2f, 0.5f};    // 視点[m] (View point)
    float hpr[3] = {121.0f, -10.0f, 0.0f}; // 視線[°] (View direction)
    dsSetViewpoint(xyz, hpr);              // 視点と視線の設定 (Set View point and direction)
    dsSetSphereQuality(3);
    dsSetCapsuleQuality(6);
}

void ODE::simloop(int pause)
{
    // reinterpret_cast<HaptHexa*>(hapthexa)->loop();
    loop_(pause);
    dSpaceCollide(space, 0, &nearcallback);
    dWorldStep(world, 5e-3);
    dJointGroupEmpty(contactgroup);
}

void ODE::command(int cmd) {
    (void) cmd;
    // reinterpret_cast<HaptHexa*>(hapthexa)->command(cmd);
}

ODE::ODE(std::function<void(int)> loop) {
    loop_ = loop;

    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simloop;
    fn.command = &command;
    fn.path_to_textures = "/usr/local/lib/drawstuff.textures/";
    
    dInitODE();
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetCFM(world, 1e-3); // CFMの設定 (global CFM)
    dWorldSetERP(world, 0.9);  // ERPの設定 (global ERP)
}

void ODE::spin() {
    dsSimulationLoop(1, nullptr, 1280, 720, &fn);
}

dWorldID ODE::getWorld() {
    return world;
}
dSpaceID ODE::getSpace() {
    return space;
}

ODE::~ODE() {
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    loop_.~function();
}