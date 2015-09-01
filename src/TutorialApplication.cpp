/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
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

#include "TutorialApplication.h"
#include <OgreTextureManager.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreHardwareBuffer.h>
#include <OgreMath.h>
#include <OgreGpuProgram.h>
#include <OgreGpuProgramManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreHighLevelGpuProgram.h>

using namespace Ogre;

static const String test_glsl_vp = "uniform mat4 worldViewProj;\n\
varying vec3 vertexPos;\n\
varying vec2 outUV;\n\
void main()\n\
{\n\
gl_Position = worldViewProj * gl_Vertex;\n\
outUV = gl_MultiTexCoord0.xy;\n\
vertexPos = normalize(gl_Vertex.xyz);\n\
}";

static const String test_glsl_fp = "uniform samplerCube cubemapTexture;\n\
   \n\
   vec4 cubeToLatLon(samplerCube cubemap, vec2 uvCoords)  \n\
   {  \n\
      const float PI = 3.141592653589793238462643383;\n\
      vec3 texCoords;\n\
      //float rx = (uvCoords.x * 2.0) - 1.0;\n\
      //float ry = (uvCoords.y * 2.0) - 1.0;\n\
      float rx = uvCoords.x;\n\
      float ry = uvCoords.y;\n\
      //texCoords.x = -cos(rx * PI * 2.0) * cos(ry * PI);\n\
      //texCoords.y = -sin(ry * PI);\n\
      //texCoords.z = sin(rx * PI * 2.0) * cos(ry * PI);\n\
      texCoords.x = -sin(rx * PI * 2.0) * sin(ry * PI);\n\
      texCoords.y = cos(ry * PI);\n\
      texCoords.z = -cos(rx * PI * 2.0) * sin(ry * PI); \n\
      return textureCube(cubemap, texCoords);  \n\
   }  \n\
   varying vec3 vertexPos;\n\
   varying vec2 outUV;\n\
   \n\
   void main( void )\n\
   {\n\
         //gl_FragColor = textureCube(cubemapTexture,vertexPos);\n\
         //gl_FragColor.rgb = vertexPos.xyz;\n\
         //gl_FragColor.r = outUV.x;\n\
         //gl_FragColor.g = outUV.y;\n\
         gl_FragColor = cubeToLatLon(cubemapTexture, outUV);\n\
   }";

//---------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//---------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//---------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
   Light *sun = mSceneMgr->createLight("Sun");
   SceneNode *sunNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("SunNode");
   sunNode->setPosition(300, 300, 300);
   sunNode->attachObject(sun);

   // PLANET GROUND
   mPlanetEntity = mSceneMgr->createEntity("Planet", SceneManager::PT_SPHERE);
   mPlanetEntity->setMaterialName("BaseWhite");

   mPlanetSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("PlanetNode");
   mPlanetSceneNode->attachObject(mPlanetEntity);
   mPlanetSceneNode->setScale(0.25, 0.25, 0.25);
   mPlanetSceneNode->setPosition(30, 0, 0);
   mPlanetSceneNode->rotate(Vector3::UNIT_Y, Radian(Math::PI));

   // PLANE
   mPlaneEntity = mSceneMgr->createEntity("Plane", SceneManager::PT_PLANE);
   mPlaneEntity->setMaterialName("BaseWhite");

   mPlaneSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("PlaneNode");
   mPlaneSceneNode->attachObject(mPlaneEntity);
   mPlaneSceneNode->setScale(0.25, 0.25, 0.25);
   mPlaneSceneNode->setPosition(-30, 0, 0);

   mPlanetMaterial = MaterialManager::getSingleton().create(
      "PlanetMaterial",
      ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
      );

   reload();

}

bool TutorialApplication::keyReleased(const OIS::KeyEvent &arg)
{
   if (arg.key == OIS::KC_L) {
      reload();
      return true;
   }

   return BaseApplication::keyReleased(arg);
}

void TutorialApplication::reload()
{
   const String planetVpName = "planet_glsl_vp";
   const String planetFpName = "planet_glsl_fp";
   const String planetVpFilename = "planet_vp.glsl";
   const String planetFpFilename = "planet_fp.glsl";

   Ogre::LogManager::getSingletonPtr()->logMessage("*** Reloading ***");

   size_t imageWidth = 1024, imageHeight = 1024;
   GpuProgramParametersSharedPtr params;
   HighLevelGpuProgramPtr vpProgram, fpProgram;
   
   Pass* pass = mPlanetMaterial->getTechnique(0)->getPass(0);

   // HAX.this is lame. have to remove from microcode cache to reload glsl
   // see GLSLLinkProgram::compileAndLink() for how the combined name is used
   // to insert into microcode cache
   String combinedName = "Vertex Program:" + planetVpName + " Fragment Program:" + planetFpName;
   if (GpuProgramManager::getSingleton().isMicrocodeAvailableInCache(combinedName)) {
      GpuProgramManager::getSingleton().removeMicrocodeFromCache(combinedName);
   }

   // load the vertex program
   vpProgram = HighLevelGpuProgramManager::getSingleton().getByName(planetVpName);
   if (vpProgram.isNull()) {
      vpProgram = HighLevelGpuProgramManager::getSingleton().
         createProgram(planetVpName,
         ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
         "glsl",
         GPT_VERTEX_PROGRAM);
      //gpuProgram->setSource(test_glsl_vp);
      vpProgram->setSourceFile(planetVpFilename);
      vpProgram->load();
   }
   else {
      vpProgram->setSourceFile(planetVpFilename);
      vpProgram->reload();
   }

   // set the vertex program
   pass->setVertexProgram(planetVpName);

   // load the fragment program
   fpProgram = HighLevelGpuProgramManager::getSingleton().getByName(planetFpName);
   if (fpProgram.isNull()) {

      fpProgram = HighLevelGpuProgramManager::getSingleton().
         createProgram(planetFpName,
         ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
         "glsl",
         GPT_FRAGMENT_PROGRAM);
      //gpuProgram->setSource(test_glsl_fp);
      fpProgram->setSourceFile(planetFpFilename);

      // set default fragment program params
      params = fpProgram->getDefaultParameters();
      params->setNamedConstant("cubemapTexture", (int)0);
      fpProgram->load();
   }
   else {
      fpProgram->setSourceFile(planetFpFilename);
      fpProgram->reload();
   }

   // set the fragment program
   pass->setFragmentProgram(planetFpName);

   if (!pass->getTextureUnitState("CubeMapTexture")) {
      pass->createTextureUnitState("CubeMapTexture");
      pass->getTextureUnitState(0)->setTextureName("TestCubemap.dds");
   }

   // set vertex program params
   params = pass->getVertexProgramParameters();
   params->setNamedAutoConstant("worldViewProj", GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
   
   //pass->_load();

   //mPlanetMaterial->reload();

   mPlaneEntity->setMaterialName("PlanetMaterial");
   mPlanetEntity->setMaterialName(mPlanetMaterial->getName());
}


//---------------------------------------------------------------------------

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
            app.go();
        } catch(Ogre::Exception& e)  {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox(NULL, e.getFullDescription().c_str(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occurred: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif

//---------------------------------------------------------------------------
