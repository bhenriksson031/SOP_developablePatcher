// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/tutorial/tutorial.h"

namespace embree
{
  extern "C" {
    int g_spp = 1;
    bool g_accumulate = 1;
  }
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("pathtracer",FEATURE_RTCORE)
    {
      registerOption("spp", [] (Ref<ParseStream> cin, const FileName& path) {
          g_spp = cin->getInt();
        }, "--spp: sets number of samples per pixel");

      registerOption("accumulate", [] (Ref<ParseStream> cin, const FileName& path) {
          g_accumulate = cin->getInt();
        }, "--accumulate <bool>: accumulate samples (on by default)");
    }
    
    void postParseCommandLine() 
    {
      /* load default scene if none specified */
      if (scene->size() == 0 && sceneFilename.ext() == "") {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}