# -*- coding: utf-8 -*-
"""
Created on Thu Sep 29 21:06:27 2022

@author: Carlos
"""
import struct
import paho.mqtt.client as mqtt
import json
import base64
import csv
from datetime import datetime
import time

import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# MQTT broker config
MQTT_PUBLISH_TOPIC = "peopleCounter"

counter = 0

token = "ZrrP3Zf_exFUjK1QKtwmTMDIKAud_Oknat8vA5y-JDflDMptOb2l7wvARz6_7X2PDcQipyDmjJeyDhqYPTxXTQ=="
org = "darosa.cdaniel@gmail.com"
url = "https://europe-west1-1.gcp.cloud2.influxdata.com"
write_client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
bucket="Esp32Lora-PresenceSensor"

# Define the write api
write_api = write_client.write_api(write_options=SYNCHRONOUS)

print("Complete. Return to the InfluxDB UI.")


# gives connection message
def on_connect(mqttc, mosq, obj, rc):
  print("Connected with result code:" + str(rc))
  # subscribe for device (aqui precisa coloca o devUi do dispositivo)
  mqttc.subscribe(
    'application/06a9736c-7cf8-4279-978f-3e78d34db32e/device/2d829326f3bfcef6/event/up'
  )


# gives message from device
def on_message(mqttc, obj, msg):
  global counter

  x = json.loads(msg.payload.decode('utf-8'))
  payload_raw = x["data"]
  payload_bytes = base64.b64decode(payload_raw)

  # We received bytes we need to convert into something usable
  measurement = int.from_bytes(payload_bytes, byteorder='big') 
  
  if(measurement == 255):
    measurement = -1

  ## InfluxDB logic
  if (measurement != 0):
    counter += measurement
    print(counter)
    point = Point(MQTT_PUBLISH_TOPIC).field("PleopleCounter", counter )
    write_api.write(bucket=bucket, record=point)

mqttc = mqtt.Client()
# Assign event callbacks
mqttc.on_connect = on_connect
mqttc.on_message = on_message
#username = 'loradatauser'
#password = '220volts#'
#mqttc.username_pw_set(username, password)
mqttc.connect("181.215.134.205", 1883, 60)

mqttc.loop_forever()