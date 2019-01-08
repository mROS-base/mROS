import os
import json
from jinja2 import Template, Environment, FileSystemLoader

std_msgs ={
	"std_msgs/String.h": 1,
	"std_msgs/UInt16.h": 10,
	"std_msgs/UInt8.h" : 3,
	"std_msgs/UInt32.h": 6
}

msg_sizes = {
	"string":0,
	"int8": 1,"uint8": 1,
	"int16": 2,"uint16": 2,
	"int32": 4,"uint32": 4,
	"int64": 8,"uint64": 8
}

msg_cpp_types = {
	"string":"string",
	"int8": "int8_t","uint8": "uint8_t",
	"int16": "int16_t","uint16": "uint16_t",
	"int32": "int32_t","uint32": "uint32_t",
	"int64": "int64_t","uint64": "uint64_t"
}

fileDir = os.path.dirname(__file__) 
os.chdir(fileDir)

included_std_msgs = []

msgs = []

i_id = 100
with open('including_msgs.json','r') as f:
	json_data = json.load(f)
	catkin_include_path = json_data['catkin_ws_dir'] + "/devel/include/"
	for line in json_data['including_msgs']:
		line = line.strip()
		if line in std_msgs:
			included_std_msgs.append({'name':line, 'id':std_msgs[line]})
		elif os.path.isfile(catkin_include_path + line):
			strNum = 0
			with open(catkin_include_path +line,'r') as h_f:
				arr = h_f.readlines()
				for i,h_line in enumerate(arr):
					if 'MD5Sum' in h_line:
						md5 = arr[i+4].strip().lstrip('	return "').rstrip('";')
					if 'Definition' in h_line:
						def_i = i+4
						msg_def = []
						while True:
							if arr[def_i + 1].strip() == "}":
								break
							else:
								msg_def_str = arr[def_i].strip().replace('return "','')[:-3]
								msg_def_arr = msg_def_str.split(' ')
								if msg_def_arr[0] == "string":
									cppType = "string"
									strNum= strNum + 1
								else:
									cppType = msg_cpp_types[msg_def_arr[0]]
								msg_def.append({
									'rosType':msg_def_arr[0],
									'cppType':cppType,
									'typeName':msg_def_arr[1],
									'size':msg_sizes[msg_def_arr[0]]
									})
								def_i = def_i + 1
					if 'DataType' in h_line:
						msg_type = arr[i+4].strip().lstrip('	return "').rstrip('";')
			line_arr = line.strip().split('/')
			line_arr[1] = line_arr[1].rstrip('.h')
			msgs.append({
			'name':line_arr[1],
			'pkg':line_arr[0],
			'NAME':line_arr[1].upper(),
			'PKG':line_arr[0].upper(),
			'id':i_id,
			'md5': md5,
			'def': msg_def,
			'type': msg_type,
			'strNum': strNum})
			i_id = i_id + 1
		else:
			raise Exception('msg header file "' + line + '" not found.')

#header generator
for msg in msgs:
	env = Environment(loader=FileSystemLoader('.'))
	template = env.get_template('msg_header_template.tpl')
	datatext = template.render({"msg":msg, "strNum":strNum})
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