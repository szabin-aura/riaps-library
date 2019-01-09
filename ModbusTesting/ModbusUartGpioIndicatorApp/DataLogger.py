#DataLogger.py
from riaps.run.comp import Component
import os
import logging
from influxdb import InfluxDBClient
from influxdb.client import InfluxDBClientError
import json
from datetime import datetime

BATCH_SIZE = 60

class DataLogger(Component):
    def __init__(self,db_host,db_port,db_name,db_user,db_password):
        super(DataLogger, self).__init__()	        
        self.pid = os.getpid()
        self.logger.info("(PID %s) - starting DataLogger",str(self.pid))
        self.point_values = []
        self.client = InfluxDBClient(host=db_host, port=db_port,
            database=db_name, username=db_user, password=db_password)        

    def on_rx_modbusData(self):
        msg = self.rx_modbusData.recv_pyobj()
        self.logger.info("PID (%s) - on_rx_modbusData():%s",str(self.pid),str(msg))
        # MM TODO:  update based on what is being logged and how we want to use it
        '''    
        timestamp = int(1e9 * data['timestamp'])
        # alternative:
        # timestamp = datetime.utcfromtimestamp(data['timestamp'])  
        self.point_values.append({
            "time": timestamp,
            "measurement": "pmu",
            "fields":  {
                "VAGA": data["VAGA"],
                "VASA": data["VASA"],
                "VASM": data["VASM"],
                "VAGM": data["VAGM"],
            },
            "tags": {
                "Actor" : "ModbusIOActor"
            },
        })
        '''
        if len(self.point_values) >= BATCH_SIZE:
            self.client.write_points(self.point_values)
            self.point_values = []
    
    def __destroy__(self):			
        self.logger.info("(PID %s) - stopping DataLogger",str(self.pid))   	        	        
