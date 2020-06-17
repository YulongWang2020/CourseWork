my_result = "output.txt"
answer = 'sudokus_finish.txt'

answer_file = open(answer, "r")
my_result_file = open(my_result, "r")
answers = answer_file.read()
results = my_result_file.read()
answer_list = []
my_result_list = []
# Solve each board using backtracking
for line in answers.split("\n"):
	if len(line) < 9:
		continue
	answer_list.append(line)

for line in results.split("\n"):

    if len(line) < 9:
    	continue
    my_result_list.append(line)
for i in range(len(answer_list)):
	
	print(my_result_list[i])
	print(answer_list[i])
	print(my_result_list[i] == answer_list[i])
answer_file.close()
my_result_file.close()