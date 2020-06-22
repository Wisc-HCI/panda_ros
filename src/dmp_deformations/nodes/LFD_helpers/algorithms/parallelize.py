from multiprocessing import Process, Pool
import time
import signal

def run_process_list(target, process_args_list, process_kwargs_list, num_procs = 4):
    for ii in range(int(len(process_args_list)/num_procs) + 1):
        processes = []
        for process_args, process_kwargs in zip(process_args_list[num_procs*ii:num_procs*(ii + 1)],
                                                process_kwargs_list[num_procs*ii:num_procs*(ii + 1)]):
            p = Process(target=target, args = process_args, kwargs=process_kwargs)
            p.start()
            processes.append(p)

        flag = False
        try:
            while not flag:
                flag = True
                for process in processes:
                    flag = flag and not process.is_alive()
                time.sleep(1)
            print 'processes ', num_procs*ii + 1, 'through ', num_procs*(ii + 1), 'complete'
        except KeyboardInterrupt:
            for process in processes:
                process.terminate()
            print 'killing all processes'

            for process in processes:
                process.join()
            raise KeyboardInterrupt

        for process in processes:
            process.join()


if __name__ == '__main__':
    def test_func(i, j = 0):
        time.sleep(3)
        print(i, '__', j)

    run_process_list(test_func, [(1,), (2,)]*7,[{'j': 3}, {'j': 3}]*7)




