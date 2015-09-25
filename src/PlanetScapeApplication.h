/*
-----------------------------------------------------------------------------
Filename:    PlanetScapeApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
Tutorial Framework (for Ogre 1.9)
http://www.ogre3d.org/wiki/
-----------------------------------------------------------------------------
*/

#ifndef __PlanetScapeApplication_h_
#define __PlanetScapeApplication_h_

#include "BaseApplication.h"
#include "rapidjson/document.h"

//---------------------------------------------------------------------------
class PlanetScapeApplication : public BaseApplication
{
public:
    PlanetScapeApplication(void);
    virtual ~PlanetScapeApplication(void);

protected:
    virtual void createScene(void);

    virtual bool keyReleased(const OIS::KeyEvent &arg);

    void reload();
    void reloadShader();
    void reloadPlanetJson();

    Ogre::Entity *mPlanetEntity;
    Ogre::Entity *mPlaneEntity;

    Ogre::MaterialPtr mPlanetMaterial;

    Ogre::SceneNode *mPlanetSceneNode;
    Ogre::SceneNode *mPlaneSceneNode;

private:
   rapidjson::Document mJson;

   void SetLandParamsFromJson(Ogre::GpuProgramParametersSharedPtr params);
   void SetNoiseLayerParamsFromJson(Ogre::GpuProgramParametersSharedPtr params);
   void SetColorTableParamsFromJson(Ogre::GpuProgramParametersSharedPtr params);
   void SetWaterParamsFromJson(Ogre::GpuProgramParametersSharedPtr params);

   int GetNoiseTypeFromString(std::string &val);
   int GetBlendTypeFromString(std::string &val);
};

//---------------------------------------------------------------------------

#endif // #ifndef __PlanetScapeApplication_h_

//---------------------------------------------------------------------------
