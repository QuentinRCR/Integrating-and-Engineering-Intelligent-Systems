import roslibpy
import spacy
import numpy as np
import time

def connect_to_ros_bridge():
    client = roslibpy.Ros(host='172.16.1.105', port=9090)
    client.run()
    print('Is ROS connected?', client.is_connected)
    return client

# Published the understood action to the corresponding topic
def send_action(direction, distance):
    translation = {"droite":'right',"gauche":'left',"avancer":'forward',"reculer":'backward',"stopper":'forward'}
    direction = translation[direction] # translate the words from french to english 

    # create the published object
    message_to_publish = {
        'command': "move",
        'args':{
            'type': direction,
            'value':distance
        }
    }

    command_talker.publish(message_to_publish)

    print(f'Moving {direction} from a distance of {distance} meters' if direction in ["forward","backward"] else f'Turning {distance} degrees toward the {direction}')

# get the original word from the list of synonyms if it exists
def get_original_word(synonym):
    if synonym in synonyms_dict.keys():
        return synonym
    for original_word, synonyms in synonyms_dict.items():
        if synonym in synonyms:
            return original_word
    return None

# add the list of synonym to the list of detectable commands
def add_synonyms_to_commands():
    command_copy = list_commands[:]
    for list_syn in synonyms_dict.values():
        command_copy += list_syn
    return set(command_copy)

# transform numbers from string to integer
def transform_to_number(value):
    if value.isdigit():
        return int(value)
    else:
        match value.lower():
            case 'un':
                return 1
            case 'deux':
                return 2
            case 'trois':
                return 3
            case 'quatre':
                return 4
            case 'cinq':
                return 5
            case 'six':
                return 6
                

# handle sentences such as "Fait trois objets de type 2"
def handle_object_number(radical,tokens):
    # if no number and no object are specified, create the object 0 once
    default_object_id = 0
    default_object_number = 1

    # get the type of each word
    pos_words = [token.pos_ for token in tokens]
    number_numbers = pos_words.count('NUM') #get the number of objects
    object_index = radical.index('objet') #get the position of the word "object"

    if number_numbers >0: # if no number were provided in the command, ignore it
        num1_index = pos_words.index('NUM')

        # if there only is one number in the sentence
        if number_numbers == 1:
            number = tokens[num1_index]

            # if the number is after the object, its the id, otherwise it is the number
            if object_index<num1_index:
                default_object_id = transform_to_number(number.text)
            else:
                default_object_number = transform_to_number(number.text)

        # if there is 2 numbers in the sentence
        elif (number_numbers == 2):
            num2_index = pos_words[num1_index+1:].index('NUM')+ num1_index + 1

            # if one number is before and the other is after, set the corresponding values
            if (num1_index < object_index < num2_index):
                default_object_id = transform_to_number(tokens[num2_index].text)
                default_object_number = transform_to_number(tokens[num1_index].text)

        message_to_publish = {
            'command': "action",
            'args':{
                'type': "make",
                'value':default_object_id
            }
        }
        command_talker.publish(message_to_publish)
        
        print(f'Creating {default_object_number} objet{"s" if default_object_number>1 else ""} with id {default_object_id}')

# Hand the messages to toggle the signs
def handle_sign_toggle(detected_command):
    message_to_publish = {
        'command': "toggle_sign",
        'args':{
            'type': "off", #by default off
            'value':0.0
        }
    }
    if(detected_command=="activer"):
        message_to_publish['args']['type'] = "on"

    command_talker.publish(message_to_publish)

# extract the value of the movement command
def get_distance(detected_command,tokens,radical):
    if detected_command in ["avancer","reculer"]:
        distance = 0.1 # go forward by 10 centimeters by default
    else:
        distance = 90 # turn 90° by default

    # if there is a number in the sentence, extract it
    pos_words = [token.pos_ for token in tokens]
    number_numbers = pos_words.count('NUM')
    if(number_numbers == 1):
        number = str(tokens[pos_words.index('NUM')]).replace(",",".") # replace , by . to be able to convert it to float
        distance = float(number)

        # convert usual units
        if len(set(["centimètres","cm","centimètre"]).intersection(radical))>0 : #if the word centimeter in present, convert the value
            distance/=100
        if len(set(["radians","rd","radian"]).intersection(radical)) : #if the word radian in present, convert the value
            distance*=180/np.pi
    return distance

# handle messages published in /sprecognition 
def speech_callback(msg):
    print(f'received message: {msg["data"]}')

    recognized_text = msg['data'] # extract the string

    tokens = nlp(recognized_text)  # tokenise it

    radical = [token.lemma_ for token in tokens] # transform to radicals 

    #get intersection of the list of commands and the radicals
    detected_commands = set(radical).intersection(list_all_commands) 
    
    # if there is a matching command
    if len(detected_commands)>0: 
        detected_command = detected_commands.pop()

        # get the original word if it was one of the synonyms
        detected_command = get_original_word(detected_command)

        # check if this command have a follow up command
        if detected_command in follow_up_command:
            detected_follow_ups = set(radical).intersection(follow_up_command[detected_command]) # extract the word that have a follow up

            # if there is a follow up command, extract it
            if len(detected_follow_ups)>0:
                detected_follow_up = detected_follow_ups.pop()
                print(f'Follow up command is: {detected_follow_up}')
                if detected_follow_up=='objet':
                    handle_object_number(radical,tokens)

        if detected_command in ["droite","gauche","avancer","reculer","stopper"]: #handled commands
            if(detected_command != "stopper"):
                distance = get_distance(detected_command,tokens,radical)
            else:
                distance = 0.0001 # replace a stop by a command to go forward a very small distance
            send_action(detected_command,distance)

        elif(detected_command in ["activer","désactiver"]):
            handle_sign_toggle(detected_command)
            print(f'Received command {detected_command}')

        else:
            print(f'Command detected: {detected_command}') #not handled commands


##==========================PARAMETERS=====================================

# elements to detect
list_commands = ["reculer","avancer","stopper","gauche","droite","faire","transporter","activer","désactiver"]
list_actions = ['prendre','déposer','objet']
list_products = ['bâton','planche']

# synonym dictionary
synonyms_dict = {
    "avancer":['devant','avant'],
    'stopper': ["arrêter","arrêt"],
    'activer': ["allumer","active"],
    'désactiver': ["éteindre","désactive"],
    'faire': ['effectuer','fabriquer'],
    'transporter': ['déplacer'],
    'gauche': ['bâbord'],
    'droite': ['tribord'],
    "reculer": ["arrière"]
}

follow_up_command = {'faire': list_actions, 'transporter':list_products}
##==========================================================================


# Initialise command recognition
list_all_commands = add_synonyms_to_commands()
nlp = spacy.load('fr_core_news_md') # load french library

# connect to the ros topics 
client = connect_to_ros_bridge()
image_listener = roslibpy.Topic(client, '/sprecognition', 'std_msgs/String', queue_size=1)
image_listener.subscribe(speech_callback)
command_talker = roslibpy.Topic(client, '/audio_command', 'project/command')

try:
    while client.is_connected:
        time.sleep(1) # analyse the request as long as the client is connected
except KeyboardInterrupt:
    client.terminate()
