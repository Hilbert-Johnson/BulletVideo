import os

def doubleFaceGenerate(input_file, output_file):
    with open(output_file, "w") as file_write:
        with open(input_file, "r") as file_reader:
            faceList = []
            lines = file_reader.readlines()
            for line in lines:
                file_write.write(line)
                if line[0] ==  'f':
                    curLists = line.split(" ")
                    faceList.append((curLists[2], curLists[1], curLists[3]))
            for faceElem in faceList:
                file_write.write("f " + faceElem[0] + " " + faceElem[1] + " " + faceElem[2])

for i in range(3000):
    input_file = os.path.join("./clothHuman/cloth/Moving", "step{}.obj".format(i))
    output_file = os.path.join("./clothHuman/cloth", str(i)+".obj")
    doubleFaceGenerate(input_file, output_file)