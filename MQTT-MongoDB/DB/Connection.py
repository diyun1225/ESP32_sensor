import pymongo
client = pymongo.MongoClient(
    'mongodb+srv://Wind_Crazy:windcrazy999@cluster0.vh07jlb.mongodb.net/?retryWrites=true&w=majority'
    )  # Write your Atlas Credentials

dbITI = client["ITI"]
