from DB.Connection import dbITI

studentColl = dbITI.student
counterIDColl = dbITI.IDCounter
studentsArr = []


def calcStudentID():
    return (counterIDColl.find_one()["studentID"]+1)

def incStuentID():
    counterIDColl.find_one_and_update(
        {'_id': 1}, {"$inc": {"studentID": 1}})


def getAllStudents():
    studentsArr.clear()
    for student in studentColl.find():
        studentsArr.append(student)
    return studentsArr


def insertstudent(name: str, age: int, track: str, courses: list):
    if(isinstance(name, str) and isinstance(age, int)
       and isinstance(track, str) and isinstance(courses, list)):
        print("Student Data valid")
    else:
        print("Error Data not valid")
        return

    studentObj = {
        # '_id': calcStudentID(),
        'name': name,
        'age': age,
        'track': track,
        'courses': courses
    }
    studentColl.insert_one(studentObj)
    studentsArr.append(studentObj)
    # incStuentID()
    print("student Data inserted successfully")


def deleteAllStudents():
    studentColl.delete_many({})
    print("All Students Data Deleted successfully")
