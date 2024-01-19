import Components.student as student
import Components.instructor as instructor
import pprint

pp = pprint.PrettyPrinter(indent=2, width=50, compact=True, sort_dicts=False)

# instructor.insertInstructor("Aya", 28, "IOT", True)
# student.insertstudent("Mina", 25, "IOT", ["Java", "C", "Python"])

# instructor.deleteAllInstructors()
# student.deleteAllStudents()

print("===========Instructors Data===========")
pp.pprint(instructor.getAllInstructors())
print("============================================")

print("===========Students Data===========")
pp.pprint(student.getAllStudents())
print("============================================")