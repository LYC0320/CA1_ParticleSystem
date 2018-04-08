#ifndef MASS_SPRING_SYSTEM_H
#define MASS_SPRING_SYSTEM_H

#include <vector>
#include <string>
#include "Vector3d.h"
#include "CParticle.h"
#include "CSpring.h"
#include "GoalNetModel.h"
#include "BallModel.h"

using std::vector;

class CMassSpringSystem
{
    public:
        enum
        {
            EXPLICIT_EULER = 0 ,
            RUNGE_KUTTA
        };

        CMassSpringSystem();
        CMassSpringSystem(const std::string &a_rcsConfigFilename);
        CMassSpringSystem(const CMassSpringSystem &a_rcMassSpringSystem);
        ~CMassSpringSystem();
        
        void Reset();
        void SimulationOneTimeStep();

        int BallNum();
        void CreateBall();

        void Draw();
        void DrawShadow(const Vector3d &a_rLightPos);
        void DrawShadowPolygon(
            const Vector3d &a_rLightPos,
            const Vector3d &a_rNormalVec,
            const Vector3d &a_rVerPos1,
            const Vector3d &a_rVerPos2
            );

        void SetSpringCoef(
            const double a_cdSpringCoef, 
            const CSpring::enType_t a_cSpringType
            );
        void SetDamperCoef(
            const double a_cdDamperCoef, 
            const CSpring::enType_t a_cSpringType
            );
        double GetSpringCoef(const CSpring::enType_t a_cSpringType);
        double GetDamperCoef(const CSpring::enType_t a_cSpringType);

        bool CheckStable();

        inline void SetRotate(const double a_cdRotate){m_dRotate = a_cdRotate;}
        inline void SetOffset(const Vector3d &a_rOffset){m_Offset = a_rOffset;}
        inline void SetDrawParticle(const bool a_bDrawParticle){ m_bDrawParticle = a_bDrawParticle; }
        inline void SetDrawStruct(const bool a_bDrawStruct){m_bDrawStruct = a_bDrawStruct;}
        inline void SetDrawShear(const bool a_bDrawShear){m_bDrawShear = a_bDrawShear;}
        inline void SetDrawBending(const bool a_bDrawBending){m_bDrawBending = a_bDrawBending;}
        inline void SetDeltaT(const double a_cdDeltaT){m_dDeltaT = a_cdDeltaT;}
        inline void SetIntegratorType(const int a_ciIntegratorType){m_iIntegratorType = a_ciIntegratorType;}
        inline void SetStartSimulation(){m_bSimulation = true;}
        inline void SetPauseSimulation(){m_bSimulation = false;}
        inline bool IsSimulation(){return m_bSimulation;}
        inline int GetIntegratorType(){return m_iIntegratorType;}
        inline double GetDeltaT(){return m_dDeltaT;}

private:

    bool m_bDrawParticle;
    bool m_bDrawStruct;      //struct stands for structural
    bool m_bDrawShear;
    bool m_bDrawBending;
    bool m_bSimulation;      //start or pause

    int m_iIntegratorType;

    double m_dDeltaT;            //delta t    
    double m_dSpringCoefStruct;
    double m_dSpringCoefShear;
    double m_dSpringCoefBending;
    double m_dDamperCoefStruct;
    double m_dDamperCoefShear;
    double m_dDamperCoefBending;
    double m_dRotate;

    Vector3d m_Offset;
    Vector3d m_ForceField;      //external force field

    // to be removed
    vector<Vector3d> m_ParticleContactForce;
    vector<Vector3d> m_BallContactForce;

    GoalNet m_GoalNet;
    vector<Ball> m_Balls;

    void ResetAllForce();
    void ResetContactForce();

    void ComputeAllForce();         //compute force of whole systems
    void ComputeParticleForce();
    void ComputeBallForce();

    void HandleCollision();
    void NetPlaneCollision();
    void BallPlaneCollision();
    void BallToBallCollision();
    void BallNetCollision();

    void Integrate();
    void ExplicitEuler();
    void RungeKutta();
    void ParticleRungeKutta();       // to be removed
    void BallRungeKutta();           // to be removed


    void DrawTriangle(CParticle &p1, CParticle &p2, CParticle &p3, const Vector3d color);

    void DrawGoalNet();
    void DrawBall();

    Vector3d CalcTriangleNormal(CParticle &p1, CParticle &p2, CParticle &p3);

};

#endif