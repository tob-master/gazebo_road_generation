import subprocess
import glob


filenames = glob.glob("*.json")


for filename in filenames:
	

	frame = filename[:-5] 


	cmd = "labelme_json_to_dataset " + frame + ".json -o " + frame + "_json"

	process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
	process.wait()

	cmd = "cp "+frame+"_json/label_viz.png label_viz/"+frame+".png" 

	process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
	process.wait()

	cmd = "cp "+frame+"_json/label.png label/"+frame+".png" 

	process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
	process.wait()

	cmd = "rm -R "+frame+"_json" 

	process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
	process.wait()

	#print process.returncode
