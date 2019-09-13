import os
import shutil
import sys


# for curve radius in meter * 2 for x y size

gazebo_path = "/home/tb/.gazebo/models/"


###########################
model_name  = "left_curve_r_200cm"
image_path  = "/home/tb/gazebo_road_generation/scripts/"
size_x      = 4
size_y      = 4
###########################


model_path               = gazebo_path + model_name + "/"

model_config_name        = model_name
model_config_description = model_name


model_sdf_name            = model_name
model_material_fname      = model_name + ".material"

model_sdf_image      = model_name
model_material_name  = model_name
model_material_image = model_name + ".png"
image_name           = model_name

model_config_txt =  """<?xml version="1.0" encoding="UTF-8"?>
<model>
   <name>""" + model_config_name + """</name>
   <version>1.0</version>
   <sdf version="1.4">model.sdf</sdf>
   <description>""" + model_config_description + """</description>
</model>\"
"""

model_sdf_txt = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.4">
   <model name=\"""" + model_sdf_name + """\">
      <static>true</static>
      <link name="link">
         <collision name="collision">
            <geometry>
               <plane>
                  <normal>0 0 1</normal>
                  <size>""" + str(size_x) + """ """ + str(size_y) + """</size>
               </plane>
            </geometry>
            <surface>
               <friction>
                  <ode>
                     <mu>100</mu>
                     <mu2>50</mu2>
                  </ode>
               </friction>
            </surface>
         </collision>
         <visual name="visual">
            <cast_shadows>false</cast_shadows>
            <geometry>
               <plane>
                  <normal>0 0 1</normal>
                  <size>""" + str(size_x) + """ """ + str(size_y) + """</size>
               </plane>
            </geometry>
            <material>
               <script>
                  <uri>model://""" + model_sdf_name  + """/materials/scripts</uri>
                  <uri>model://""" + model_sdf_name  + """/materials/textures/</uri>
                  <name>"""        + model_sdf_image + """/Image</name>
               </script>
            </material>
         </visual>
      </link>
   </model>
</sdf>
"""

# OGRE
model_material_txt = """material """ + model_material_name + """/Image
{
  technique
  {
    pass
    {
    
      scene_blend alpha_blend
    
      texture_unit
      {
        texture """ + model_material_image + """ PF_L8
	    filtering anisotropic
        max_anisotropy 16
      }
    }
  }
}
"""

def create_file(path, txt):
    
    f = open(path, "a")
    f.write(txt)
    f.close()
    
def make_dirs(path):

    os.mkdir(path)
    os.mkdir(path + "materials")
    os.mkdir(path + "materials/scripts")
    os.mkdir(path + "materials/textures")

def get_user_input():
    
    print("Path already exists!")
    print("Delete path and create new?")
    return input("yes/no?")

def delete_path(path):
    
    shutil.rmtree(path)
    print(path," deleted!")
    
    
def copy_image(image_path, model_path, name):
   
    source = image_path + name + ".png"
    target = model_path + "materials/textures/" + name + ".png"
    
    
    try:
        shutil.copyfile(source,target)
        
    except IOError as e:
        
        print("Unable to copy file. %s" % e)
        exit(1)
    
    except:
    
        print("Unexpected error:", sys.exc_info())
        exit(1)


	
if os.path.exists(model_path) and os.path.isdir(model_path):
    
    
    user_choice = get_user_input() 
    
    
    if user_choice == "yes":
        
        delete_path(model_path)

        make_dirs(model_path)
        
        create_file(model_path + "model.config", model_config_txt)
        create_file(model_path + "model.sdf", model_sdf_txt)        
        create_file(model_path + "materials/scripts/" + model_material_fname, model_material_txt)         

        copy_image(image_path, model_path, image_name)

         
        print("Model created!")
        
    else:
        print("Exit program!")

else:

    make_dirs(model_path)
    
    create_file(model_path + "model.config", model_config_txt)
    create_file(model_path + "model.sdf", model_sdf_txt)        
    create_file(model_path + "materials/scripts/" + model_material_fname, model_material_txt) 
    
    copy_image(image_path, model_path, image_name)

    print("Model created!")
