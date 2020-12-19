import datetime
import logging


def setup_logging(LOGGER, worker, worker_id):
    """
    set up logging for the process calling this function.
    :param worker: True means it is a worker process from the pool. False means it is the main process.
    :param worker_id: unique ID identifying the process
    :return:
    """
    for h in list(LOGGER.handlers):
        LOGGER.removeHandler(h)
    now = datetime.datetime.now()
    formatter = logging.Formatter('%(asctime)s - %(threadName)s - %(levelname)s - %(message)s')
    name = "compression_results_"
    if worker:
        name += "worker_"
        global CONNECTION
        CONNECTION = None
    name += str(worker_id) + '_' + now.isoformat() + '.txt'
    file_log_handler = logging.FileHandler(name)
    file_log_handler.setFormatter(formatter)
    LOGGER.addHandler(file_log_handler)

    console_log_handler = logging.StreamHandler()
    console_log_handler.setFormatter(formatter)
    LOGGER.addHandler(console_log_handler)

    LOGGER.setLevel('INFO') # DEBUG < INFO < WARNING < ERROR < CRITICAL


if __name__ == '__main__':
    pass
