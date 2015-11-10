/*
-----------------------------------------------------------------------------
Filename:    PlanetScapeApplication.cpp
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

#include "PlanetScapeApplication.h"
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
#include <unordered_map>

using namespace Ogre;

//---------------------------------------------------------------------------
PlanetScapeApplication::PlanetScapeApplication(void) :
mShutDownFileWatcher(false),
mShouldReload(false),
mThread()
{
}
//---------------------------------------------------------------------------
PlanetScapeApplication::~PlanetScapeApplication(void)
{
   mShutDownFileWatcher = true;
   if (mThread.joinable()) {
      mThread.join();
   }
}

//---------------------------------------------------------------------------
void PlanetScapeApplication::createScene(void)
{
   Light *sun = mSceneMgr->createLight("Sun");
   sun->setDiffuseColour(ColourValue(1.f, 1.f, 1.f, 1.f));

   SceneNode *sunNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("SunNode");
   sunNode->setPosition(300, 300, 300);
   sunNode->attachObject(sun);
   mSceneMgr->setAmbientLight(ColourValue(0.1, 0.1, 0.12, 1.f));

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

   mPlaneEntity->setMaterialName(mPlanetMaterial->getName());
   mPlanetEntity->setMaterialName(mPlanetMaterial->getName());

   reloadPlanetJson();
   reload();

   mThread = std::thread(&PlanetScapeApplication::FileWatcherThreadFunc, this);
}

String PlanetScapeApplication::findFilePath(const String &filename)
{
   ResourceGroupManager* mgr = ResourceGroupManager::getSingletonPtr();
   std::string resourceGroup(mgr->findGroupContainingResource(filename));
   Ogre::FileInfoListPtr fileInfos = mgr->findResourceFileInfo(resourceGroup, filename);
   FileInfoList::iterator it = fileInfos->begin();
   if (it != fileInfos->end()) {
      return it->archive->getName();
   }

   return "";
}

void PlanetScapeApplication::FileWatcherThreadFunc() 
{   
   const String filename = "planet1.json";
   String path = findFilePath(filename);

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
   // monitor directory for change
   HANDLE hDir = FindFirstChangeNotification(path.c_str(), false, FILE_NOTIFY_CHANGE_LAST_WRITE);
   if (hDir == INVALID_HANDLE_VALUE) {
      Ogre::LogManager::getSingletonPtr()->logMessage(String("Watcher received invalid handle"));
      return;
   }

   // monitor specific file
   HANDLE hFile = CreateFile((path + "/" + filename).c_str(), GENERIC_READ, FILE_SHARE_READ, NULL,
      OPEN_EXISTING, 0, NULL);

   FILETIME creationTime, lastAccessTime, oldWriteTime;
   BOOL success = GetFileTime(hFile, &creationTime, &lastAccessTime, &oldWriteTime);
   CloseHandle(hFile);
   if (!success) {
      CloseHandle(hDir);
      return;
   }

   static const DWORD timeoutMS = 1000;

   while (!mShutDownFileWatcher) {

      // don't use INFINITE, we need to periodically check if the app is shutting down
      DWORD result = WaitForSingleObject(hDir, timeoutMS);

      if (mShutDownFileWatcher) {
         break;
      }

      if (result == WAIT_OBJECT_0) {
         // sleep a bit to ignore multiple notifications
         Sleep(WAIT_TIMEOUT);

         FILETIME lastWriteTime;
         HANDLE hFile = CreateFile((path + "/" + filename).c_str(), GENERIC_READ, FILE_SHARE_READ, NULL,
            OPEN_EXISTING, 0, NULL);
         BOOL success = GetFileTime(hFile, &creationTime, &lastAccessTime, &lastWriteTime);
         CloseHandle(hFile);
         if (!success) {
            break;
         }

         if (CompareFileTime(&lastWriteTime, &oldWriteTime) != 0) {
            oldWriteTime = lastWriteTime;
            mShouldReload = true;
         }

         FindNextChangeNotification(hDir);
      }
   }

   CloseHandle(hDir);
#endif
}

bool PlanetScapeApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
   if (mShouldReload) {
      mShouldReload = false;
      reloadPlanetJson();
      reload();
   }

   return BaseApplication::frameRenderingQueued(evt);
}

bool PlanetScapeApplication::keyReleased(const OIS::KeyEvent &arg)
{
   if (arg.key == OIS::KC_L) {
      reloadPlanetJson();
      reload();
      return true;
   }

   return BaseApplication::keyReleased(arg);
}

void PlanetScapeApplication::reloadPlanetJson()
{
   const String filename = "planet1.json";
   
   DataStreamPtr data = ResourceGroupManager::getSingletonPtr()->openResource(filename, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
   if (data.isNull()) {
      Ogre::LogManager::getSingletonPtr()->logMessage(String("Couldn't find file  ") + filename);
      return;
   }

   String jsonString = data->getAsString();
   mJson.Parse(jsonString.c_str());

   if (mJson.HasParseError()) {
      Ogre::LogManager::getSingletonPtr()->logMessage(String("Parse error discovered in ") + filename);
      //Ogre::LogManager::getSingletonPtr()->logMessage(String("Parse error: ") + json.GetParseError());
      data->close();
      return;
   }

   if (mJson.HasMember("name")) {
      Ogre::LogManager::getSingletonPtr()->logMessage(String("PlanetName: ") + mJson["name"].GetString());
   }

   data->close();
}

int PlanetScapeApplication::GetNoiseTypeFromString(std::string &val)
{
   int type = 0;

   const std::unordered_map<std::string, int> types ({
      { "puffy", 0 },
      { "speckled", 1 },
      { "turbulence", 2 },
      { "ridged", 3 },
      { "iq", 4 },
      { "alien", 5 }
   });

   if (types.find(val) != types.end()) {
      return types.at(val);
   }
   else {
      return 0;
   }
}

int PlanetScapeApplication::GetBlendTypeFromString(std::string &val)
{
   int type = 0;

   const std::unordered_map<std::string, int> types({
      { "add", 0 },
      { "multiply", 1 }
   });

   return types.find(val) == types.end() ? 0 : types.at(val);
}

void PlanetScapeApplication::SetLandParamsFromJson(GpuProgramParametersSharedPtr params)
{
   if (mJson["land"].HasMember("continents")) {
      params->setNamedConstant("numContinents", mJson["land"]["continents"].GetInt());
   }
   else {
      params->setNamedConstant("numContinents", (int)0);
   }

   if (mJson["land"].HasMember("equator_offset")) {
      params->setNamedConstant("equator_offset", (float)mJson["land"]["equator_offset"].GetDouble());
   }
   else {
      params->setNamedConstant("equator_offset", 0.f);
   }

   if (mJson["land"].HasMember("pole_size")) {
      params->setNamedConstant("pole_size", (float)mJson["land"]["pole_size"].GetDouble());
   }
   else {
      params->setNamedConstant("pole_size", 0.f);
   }

   if (mJson["land"].HasMember("pole_noise_amount")) {
      params->setNamedConstant("pole_noise_amount", (float)mJson["land"]["pole_noise_amount"].GetDouble());
   }
   else {
      params->setNamedConstant("pole_noise_amount", 0.f);
   }

   if (mJson["land"].HasMember("pole_hardness")) {
      params->setNamedConstant("pole_hardness", (float)mJson["land"]["pole_hardness"].GetDouble());
   }
   else {
      params->setNamedConstant("pole_hardness", 0.f);
   }

   SetNoiseLayerParamsFromJson(params);
   SetColorTableParamsFromJson(params);
}

void PlanetScapeApplication::SetNoiseLayerParamsFromJson(GpuProgramParametersSharedPtr params)
{
   if (mJson["land"].HasMember("noise_layers")) {
      const rapidjson::Value& noiseLayers = mJson["land"]["noise_layers"];
      int numLayers = noiseLayers.Size();
      params->setNamedConstant("numNoiseLayers", numLayers);

      // this should be a multiple of four because Ogre prefers that
      const uint maxLayers = 4;

      int noiseLayerTypes[maxLayers];
      float noiseLayerSeeds[maxLayers];
      float noiseLayerMinHeights[maxLayers];
      float noiseLayerMaxHeights[maxLayers];
      int noiseLayerBlendTypes[maxLayers];

      for (rapidjson::SizeType i = 0; i < numLayers && i < maxLayers; ++i) {
         if (noiseLayers[i].HasMember("type")) {
            noiseLayerTypes[i] = GetNoiseTypeFromString(std::string(noiseLayers[i]["type"].GetString()));
         }

         if (noiseLayers[i].HasMember("seed")) {
            noiseLayerSeeds[i] = noiseLayers[i]["seed"].GetDouble();
         }

         if (noiseLayers[i].HasMember("min_height")) {
            noiseLayerMinHeights[i] = noiseLayers[i]["min_height"].GetDouble();
         }

         if (noiseLayers[i].HasMember("max_height")) {
            noiseLayerMaxHeights[i] = noiseLayers[i]["max_height"].GetDouble();
         }

         if (noiseLayers[i].HasMember("blend")) {
            noiseLayerBlendTypes[i] = GetBlendTypeFromString(std::string(noiseLayers[i]["blend"].GetString()));
         }
      }

      params->setNamedConstant("noiseLayerTypes", noiseLayerTypes, maxLayers / 4);
      params->setNamedConstant("noiseLayerSeeds", noiseLayerSeeds, maxLayers / 4);
      params->setNamedConstant("noiseLayerMinHeights", noiseLayerMinHeights, maxLayers / 4);
      params->setNamedConstant("noiseLayerMaxHeights", noiseLayerMaxHeights, maxLayers / 4);
      params->setNamedConstant("noiseLayerBlendTypes", noiseLayerBlendTypes, maxLayers / 4);
   }
   else {
      params->setNamedConstant("numNoiseLayers", (int)0);
   }
}

void PlanetScapeApplication::SetColorTableParamsFromJson(GpuProgramParametersSharedPtr params)
{
   if (mJson["land"].HasMember("color_table")) {
      const rapidjson::Value& colorTable = mJson["land"]["color_table"];
      int numEntries = colorTable.Size();
      params->setNamedConstant("numColorTableEntries", numEntries);

      // this should be a multiple of four because Ogre prefers that
      const uint maxEntries = 8;

      float colorTableOffsets[maxEntries];
      float poleColorTableColors[maxEntries * 3];
      float equatorColorTableColors[maxEntries * 3];
      float colorTableDitherAmounts[maxEntries];

      for (rapidjson::SizeType i = 0; i < numEntries && i < maxEntries; ++i) {
         if (colorTable[i].HasMember("offset")) {
            colorTableOffsets[i] = colorTable[i]["offset"].GetDouble();
         }

         if (colorTable[i].HasMember("equator_color")) {
            equatorColorTableColors[i * 3 + 0] = colorTable[i]["equator_color"][0].GetDouble();
            equatorColorTableColors[i * 3 + 1] = colorTable[i]["equator_color"][1].GetDouble();
            equatorColorTableColors[i * 3 + 2] = colorTable[i]["equator_color"][2].GetDouble();
         }

         if (colorTable[i].HasMember("pole_color")) {
            poleColorTableColors[i * 3 + 0] = colorTable[i]["pole_color"][0].GetDouble();
            poleColorTableColors[i * 3 + 1] = colorTable[i]["pole_color"][1].GetDouble();
            poleColorTableColors[i * 3 + 2] = colorTable[i]["pole_color"][2].GetDouble();
         }

         if (colorTable[i].HasMember("dither_amount")) {
            colorTableDitherAmounts[i] = colorTable[i]["dither_amount"].GetDouble();
         }
      }

      params->setNamedConstant("colorTableOffsets", colorTableOffsets, maxEntries / 4);
      params->setNamedConstant("equatorColorTableColors", equatorColorTableColors, (maxEntries * 3) / 4);
      params->setNamedConstant("poleColorTableColors", poleColorTableColors, (maxEntries * 3) / 4);
      params->setNamedConstant("colorTableDitherAmounts", colorTableDitherAmounts, maxEntries / 4);
   }
   else {
      params->setNamedConstant("numColorTableEntries", 0);
   }
}

void PlanetScapeApplication::SetWaterParamsFromJson(GpuProgramParametersSharedPtr params)
{
   if (mJson["water"].HasMember("level")) {
      params->setNamedConstant("waterShallowLevel", (float)mJson["water"]["level"].GetDouble());

      // shallow water
      if (mJson["water"].HasMember("shallow_color")) {
         Vector3 color = Vector3::ZERO;
         color.x = (float)mJson["water"]["shallow_color"][0].GetDouble();
         color.y = (float)mJson["water"]["shallow_color"][1].GetDouble();
         color.z = (float)mJson["water"]["shallow_color"][2].GetDouble();

         params->setNamedConstant("waterShallowColor", color);
      }
      else {
         params->setNamedConstant("waterShallowColor", Vector3::ZERO);
      }

      // deep water
      if (mJson["water"].HasMember("deep_level")) {
         params->setNamedConstant("waterDeepLevel", (float)mJson["water"]["deep_level"].GetDouble());
      }
      else {
         params->setNamedConstant("waterDeepLevel", (float)0.0);
      }

      if (mJson["water"].HasMember("deep_color")) {
         Vector3 color = Vector3::ZERO;
         color.x = (float)mJson["water"]["deep_color"][0].GetDouble();
         color.y = (float)mJson["water"]["deep_color"][1].GetDouble();
         color.z = (float)mJson["water"]["deep_color"][2].GetDouble();

         params->setNamedConstant("waterDeepColor", color);
      }
      else {
         params->setNamedConstant("waterDeepColor", Vector3::ZERO);
      }

      if (mJson["water"].HasMember("frozen_color")) {
         Vector3 color = Vector3::ZERO;
         color.x = (float)mJson["water"]["frozen_color"][0].GetDouble();
         color.y = (float)mJson["water"]["frozen_color"][1].GetDouble();
         color.z = (float)mJson["water"]["frozen_color"][2].GetDouble();

         params->setNamedConstant("waterFrozenColor", color);
      }
      else {
         params->setNamedConstant("waterFrozenColor", Vector3::ZERO);
      }

      // specular
      if (mJson["water"].HasMember("specular_amount")) {
         params->setNamedConstant("waterSpecularAmount", (float)mJson["water"]["specular_amount"].GetDouble());
      }
      else {
         params->setNamedConstant("waterSpecularAmount", (float)0.0);
      }

      if (mJson["water"].HasMember("specular_color")) {
         Vector3 color = Vector3::ZERO;
         color.x = (float)mJson["water"]["specular_color"][0].GetDouble();
         color.y = (float)mJson["water"]["specular_color"][1].GetDouble();
         color.z = (float)mJson["water"]["specular_color"][2].GetDouble();

         params->setNamedConstant("waterSpecularColor", color);
      }
      else {
         params->setNamedConstant("waterSpecularColor", Vector3::UNIT_SCALE);
      }
   }
   else {
      params->setNamedConstant("waterShallowLevel", (float)0.0);
   }
}

void PlanetScapeApplication::reload()
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
      fpProgram->setSourceFile(planetFpFilename);
      fpProgram->load();
   }
   else {
      fpProgram->setSourceFile(planetFpFilename);
      fpProgram->reload();
   }

   // set the fragment program
   pass->setFragmentProgram(planetFpName);

   // set fragment program params
   if (!fpProgram.isNull()) {
      params = pass->getFragmentProgramParameters();

      params->setNamedAutoConstant("lightPositionObjectSpace", GpuProgramParameters::ACT_LIGHT_POSITION_OBJECT_SPACE);
      params->setNamedAutoConstant("cameraPositionObjectSpace", GpuProgramParameters::ACT_CAMERA_POSITION_OBJECT_SPACE);
      params->setNamedAutoConstant("lightColor", GpuProgramParameters::ACT_LIGHT_DIFFUSE_COLOUR);
      params->setNamedAutoConstant("ambientColor", GpuProgramParameters::ACT_AMBIENT_LIGHT_COLOUR);

      // set fragment program values from json file
      if (!mJson.IsNull()) {
         if (mJson.HasMember("land")) {
            SetLandParamsFromJson(params);
         }

         if (mJson.HasMember("water")) {
            SetWaterParamsFromJson(params);
         }
      }
   }


   // set vertex program params
   params = pass->getVertexProgramParameters();
   params->setNamedAutoConstant("worldViewProj", GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);

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
        PlanetScapeApplication app;

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
