from DB.Connection import dbITI
instructorColl = dbITI.instructors
counterIDColl = dbITI.IDCounter

instructosArr = []


def calcInstructorID():
    return (counterIDColl.find_one()["instructorID"]+1)


def incInstructorID():
    counterIDColl.find_one_and_update(
        {'_id': 1}, {"$inc": {"instructorID": 1}})


def getAllInstructors():
    instructosArr.clear()
    for intructor in instructorColl.find():
        instructosArr.append(intructor)
    return instructosArr


def insertInstructor(name: str, age: int, track: str, supervisor: bool):
    if(isinstance(name, str) and isinstance(age, int)
       and isinstance(track, str) and isinstance(supervisor, bool)):
        print("Instructor Data valid")
    else:
        print("Error Data not valid")
        return

    supervisor = "true" if supervisor == True or supervisor == "true" else "false"
    instructorObj = {
        '_id': calcInstructorID(),
        'name': name,
        'age': age,
        'track': track,
        'supervisor': supervisor
    }
    instructorColl.insert_one(instructorObj)
    instructosArr.append(instructorObj)
    incInstructorID()
    print("Instructor Data inserted successfully")


def deleteAllInstructors():
    instructorColl.delete_many({})
    print("All Instructors Data Deleted successfully")
