import speech_recognition as sr
import roslibpy

# listen to the mic and transform any loud sound to a text
def get_input_text():
    with sr.Microphone() as source:
        print(f'Listening ...')
        audio = recognizer.listen(source)

        try:
            recognized_text = recognizer.recognize_google(audio, language="fr-FR")
        except sr.UnknownValueError:
            print("Could not understand audio")
            recognized_text = ''
    return recognized_text

def calibrate_noise():
    print("Calibrating, don't make any noise")
    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source)  # listen for 1 second to calibrate the energy threshold for ambient noise levels

def connect_to_ros_bridge():
    client = roslibpy.Ros(host='172.16.1.105', port=9090)
    client.run()
    print('Is ROS connected?', client.is_connected)
    return client

##==========================================================================



client = connect_to_ros_bridge()

# Connect to topics
image_listener = roslibpy.Topic(client, '/sprecognition', 'std_msgs/String')

# initialise the recognizer
recognizer = sr.Recognizer()
calibrate_noise() # adapt the sound threshold to the ambient noise

while client.is_connected:
    recognized_text = get_input_text() # get the command from the mic of the computer
    if(recognized_text != ''): # if something was recognized, publish it to the /sprecognition topic
        image_listener.publish({"data": recognized_text})
    print(recognized_text) # to make sur the command was correctly understood