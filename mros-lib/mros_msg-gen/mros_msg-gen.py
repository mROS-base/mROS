import os
import json
from jinja2 import Template, Environment, FileSystemLoader

std_msgs ={
	"std_msgs/Int8.h" : 1,
	"std_msgs/Int16.h": 2,
	"std_msgs/Int32.h": 3,
	"std_msgs/Int64.h": 4,
	"std_msgs/UInt8.h" : 5,
	"std_msgs/UInt16.h": 6,
	"std_msgs/UInt32.h": 7,
	"std_msgs/UInt64.h": 8,
	"std_msgs/String.h": 9,
	"std_msgs/Int8MultiArray.h" : 11,
	"std_msgs/Int16MultiArray.h": 12,
	"std_msgs/Int32MultiArray.h": 13,
	"std_msgs/Int64MultiArray.h": 14,
	"std_msgs/UInt8MultiArray.h" : 15,
	"std_msgs/UInt16MultiArray.h": 16,
	"std_msgs/UInt32MultiArray.h": 17,
	"std_msgs/UInt64MultiArray.h": 18,
}

msg_sizes = {
	"string":0,
	"int8": 1,"uint8": 1,
	"int16": 2,"uint16": 2,
	"int32": 4,"uint32": 4,
	"int64": 8,"uint64": 8,
}

msg_cpp_types = {
	"string":"string",
	"int8": "int8_t","uint8": "uint8_t",
	"int16": "int16_t","uint16": "uint16_t",
	"int32": "int32_t","uint32": "uint32_t",
	"int64": "int64_t","uint64": "uint64_t",
	"string" : "string"
}

msgs = []
depMsgs = []
strNum = 0

i_id = 100

def typeInterpreter(msg_def_str, msgDependences):
	global strNum
	msg_def_arr = msg_def_str.split(' ')
	print(msg_def_arr)
	msgType = msg_def_arr[0]
	msgName = msg_def_arr[1]
	isArray = False
	if msgType[-2:] == "[]":
		# the type is array
		msgType = msgType[:-2]
		isArray = True
	if msgType in msg_cpp_types: 
		# the type is primitive type
		if msgType == "string":
			strNum += 1
		return {
			'rosType':msgType,
			'cppType':msg_cpp_types[msgType],
			'typeName':msgName,
			'size':msg_sizes[msgType],
			'isArray': isArray,
			'isCustomType': False
		}
	else:
		#search type from list in .json 
		for item in json_data['including_msgs']:
			if msgType in item:
				# the type is defined message type
				msgDependences.append(item)
				msgTypeArray = item[:-2].split("/")
				if isArray:
					raise Exception('the arrays of user-defined types are not supported, sorry.')
				return {
					'rosType':msgType,
					'cppType':msgTypeArray[0]+"::"+msgTypeArray[1],
					'typeName':msgName,
					'size':0,
					'isArray': isArray,
					'isCustomType': True
				}
		for pkgName in json_data['depending_packages']:
			#search from depending packages
			line = pkgName+"/"+msgType+".h"
			try:
				depMsgs.append(msgDataGenerator(line))
				msgDependences.append(line)
				return {
					'rosType':msgType,
					'cppType':pkgName +"::"+msgType,
					'typeName':msgName,
					'size':0,
					'isArray': isArray,
					'isCustomType': True
				}
			except Exception:
				continue
		raise Exception("the type "+msgType+" not found: is it generated by ROS msggen ?")

def msgDataGenerator(line):
	global i_id
	if os.path.isfile(catkin_include_path + line):
		msgDependences = []
		with open(catkin_include_path +line,'r') as h_f:
			arr = h_f.readlines()
			for i,h_line in enumerate(arr):
				if 'MD5Sum' in h_line:
					md5 = arr[i+4].strip().replace('return "','').rstrip('";')
				if 'Definition' in h_line: #getting message definition
					def_i = i+4
					msg_def = []
					while True:
						if (arr[def_i + 1].strip() == "}") or (arr[def_i].strip() == "================================================================================\\n\\"):
							break #end of message definition
						else:
							msg_def_str = arr[def_i].strip().replace('return "','')[:-3]
							msg_def.append(typeInterpreter(msg_def_str, msgDependences))
							def_i = def_i + 1
				if 'DataType' in h_line:
					msg_type = arr[i+4].strip().lstrip('	return "').rstrip('";')
		line_arr = line.strip().split('/')
		line_arr[1] = line_arr[1].rstrip('.h')
		i_id = i_id + 1
		return {
			'name':line_arr[1],
			'pkg':line_arr[0],
			'NAME':line_arr[1].upper(),
			'PKG':line_arr[0].upper(),
			'id':i_id,
			'md5': md5,
			'def': msg_def,
			'type': msg_type,
			'strNum': strNum,
			'dependences': msgDependences
		}
	else:
		raise Exception('msg header file "' + line + '" not found.')

fileDir = os.path.dirname(__file__) 
os.chdir(fileDir)

included_std_msgs = []


with open('including_msgs.json','r') as f:
	json_data = json.load(f)
	catkin_include_path = json_data['catkin_ws_dir'] + "/devel/include/"
	for line in json_data['including_msgs']:
		line = line.strip()
		if line in std_msgs:
			#use of msgs in std_msgs
			linestr = line.split('/')
			included_std_msgs.append({'pkg': linestr[0],'name':linestr[1].rstrip('.h'), 'id':std_msgs[line]})
		else:
			msgs.append(msgDataGenerator(line))

#header generator
for msg in msgs:
	env = Environment(loader=FileSystemLoader('.'))
	template = env.get_template('msg_header_template.tpl')
	datatext = template.render({"msg":msg, "strNum":strNum})
	pkgPath = '../../mros-msgs/'+msg['pkg']
	if not(os.path.isdir(pkgPath)):
		os.mkdir(pkgPath)
	with open(os.path.join(pkgPath,msg['name']+".h"),"wb") as f:
		f.write(datatext)

for msg in depMsgs:
	env = Environment(loader=FileSystemLoader('.'))
	template = env.get_template('msg_header_template.tpl')
	datatext = template.render({"msg":msg, "strNum":strNum})
	pkgPath = '../../mros-msgs/'+msg['pkg']
	if not(os.path.isdir(pkgPath)):
		os.mkdir(pkgPath)
	with open(os.path.join(pkgPath,msg['name']+".h"),"wb") as f:
		f.write(datatext)

# header_includer generator
env = Environment(loader=FileSystemLoader('.'))
template = env.get_template('msg_headers_includer.tpl')
datatext = template.render({"msgs":msgs,"std_msgs":included_std_msgs})
with open("../../mros-msgs/message_headers.h","wb") as f:
	f.write(datatext)
# header_specializer generator
env = Environment(loader=FileSystemLoader('.'))
template = env.get_template('msg_headers_spetializer.tpl')
datatext = template.render({"msgs":msgs,"std_msgs":included_std_msgs})
with open("../../mros-msgs/message_class_specialization.h","wb") as f:
	f.write(datatext)

