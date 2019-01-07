import os
import json
from jinja2 import Template, Environment, FileSystemLoader

std_msgs ={
	"std_msgs/String.h": 1,
	"std_msgs/UInt16.h": 10,
	"std_msgs/UInt8.h" : 3,
	"std_msgs/UInt32.h": 6
}

included_std_msgs = []

msgs = []

i_id = 100
with open('including_msgs.json','r') as f:
	json_data = json.load(f)
	catkin_include_path = json_data['catkin_ws_dir'] + "/devel/include/"
	for line in json_data['including_msgs']:
		line = line.strip()
		print(line)
		if line in std_msgs:
			included_std_msgs.append({'name':line, 'id':std_msgs[line]})
			print std_msgs[line]
		elif os.path.isfile(catkin_include_path + line):
			with open(catkin_include_path +line,'r') as h_f:
				arr = h_f.readlines()
				for i,h_line in enumerate(arr):
					if 'MD5Sum' in h_line:
						md5 = arr[i+4].strip().lstrip('	return "').rstrip('";')
					if 'Definition' in h_line:
						def_i = i+4
						msg_def = []
						while True:
							print(arr[def_i])
							if arr[def_i + 1].strip() == "}":
								break
							else:
								msg_def_str = arr[def_i].strip().lstrip('	return "').rstrip('";').rstrip("\\n\\")
								msg_def.append(msg_def_str.split(' '))
								def_i = def_i + 1
					if 'DataType' in h_line:
						msg_type = arr[i+4].strip().lstrip('	return "').rstrip('";')
			line_arr = line.strip().split('/')
			line_arr[1] = line_arr[1].rstrip('.h')
			print('start msg_def')
			print(msg_def)
			print('end msg_def')
			msgs.append({
			'name':line_arr[1],
			'pkg':line_arr[0],
			'NAME':line_arr[1].upper(),
			'PKG':line_arr[0].upper(),
			'id':i_id,
			'md5': md5,
			'def': msg_def,
			'type': msg_type})
			i_id = i_id + 1
		else:
			raise Exception('Error: no such header file')

print msgs
#header generator
for msg in msgs:
	env = Environment(loader=FileSystemLoader('.'))
	template = env.get_template('msg_header_template.tpl')
	datatext = template.render({"msg":msg})
	pkgPath = '../mros-msgs/'+msg['pkg']
	if not(os.path.isdir(pkgPath)):
		os.mkdir(pkgPath)
	with open(os.path.join(pkgPath,msg['name']+".h"),"wb") as f:
		f.write(datatext)

# header_includer generator
env = Environment(loader=FileSystemLoader('.'))
template = env.get_template('msg_headers_includer.tpl')
datatext = template.render({"msgs":msgs,"std_msgs":included_std_msgs})

with open("../mros-msgs/message_headers.h","wb") as f:
	f.write(datatext)