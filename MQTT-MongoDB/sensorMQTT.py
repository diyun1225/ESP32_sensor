import random
import json
import Components.student as student
import Components.sensor as sensor
from paho.mqtt import client as mqtt_client
from datetime import datetime

broker = 'broker.emqx.io'
port = 1883
topic = "mongoDB/sensors"

# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'



def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        # using now() to get current time
        current_time = datetime.today().replace(microsecond=0)

        json_current_time = {"current_time":current_time}
        # print(json_current_time)

        # parsing JSON string:
        stdObj = json.loads(msg.payload.decode())
        # appending the data
        stdObj.update(json_current_time)
        print(stdObj)

        

        sensor.insertstudent(
            stdObj["current_time"], stdObj["environment_temperature"], stdObj["environment_humidity"], stdObj["wind_speed"], stdObj["wind_direction"], stdObj["DCvoltage"],
             stdObj["DCcurrent"], stdObj["Ktemperature"])
        
        sensor.delete_old_data()
    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()

