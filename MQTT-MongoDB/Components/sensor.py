from DB.Connection import dbITI
from datetime import datetime, timedelta

studentColl = dbITI.sensor
counterIDColl = dbITI.IDCounter
studentsArr = []


def calcStudentID():
    return (counterIDColl.find_one()["sensorID"]+1)

def incStuentID():
    counterIDColl.find_one_and_update(
        {'_id': 1}, {"$inc": {"sensorID": 1}})


def getAllStudents():
    studentsArr.clear()
    for sensor in studentColl.find():
        studentsArr.append(sensor)
    return studentsArr

def delete_old_data():
    threshold_time = datetime.now() - timedelta(minutes=30)  # 設定時間閾值為2分鐘前的時間
    studentColl.delete_many({"current time": {"$lt": threshold_time}})
    
def insertstudent(current_time: datetime, environment_temperature: float, environment_humidity: float, wind_speed: float, wind_direction: float,
                DCvoltage: float, DCcurrent: float, Ktemperature: float):
    if(isinstance(current_time, datetime) and isinstance(environment_temperature, float) and isinstance(environment_humidity, float)
       and isinstance(wind_speed, float) and isinstance(wind_direction, float) and isinstance(DCvoltage, float)
       and isinstance(DCcurrent, float) and isinstance(Ktemperature, float)):
        print("Sensor Data valid")
    else:
        print("Error Data not valid")
        return

    studentObj = {
        #'_id': calcStudentID(),
        
        'environment temperature': environment_temperature,
        'environment humidity': environment_humidity,
        'wind speed': wind_speed,
        'wind direction': wind_direction,
        'DC voltage': DCvoltage,
        'DC current': DCcurrent,
        'K temperature': Ktemperature,
        'current time': current_time
    }
    studentColl.insert_one(studentObj)
    studentsArr.append(studentObj)
    # incStuentID()
    print("sensor Data inserted successfully")


def deleteAllStudents():
    studentColl.delete_many({})
    print("All sensors Data Deleted successfully")
