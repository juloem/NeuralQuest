import os


class FileUtils:
    """
    A utility class for handling file operations, such as resolving paths,
    checking file existence, and creating directories.

    Methods:
        resolve_path(file_path):
            Resolves the absolute path of a given file path.
        file_exists(file_path):
            Checks if a file exists at the specified path.
        create_directory(dir_path):
            Creates a directory if it doesn't already exist.
    """

    @staticmethod
    def resolve_path(file_path : str) -> str:
        """
        Resolves the absolute path of a given file path.

        Args:
            file_path (str): The file path to resolve.

        Returns:
            str: The absolute path to the file.
        """
        if not os.path.isabs(file_path):
            main_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            file_path = os.path.join(main_dir, file_path)
        return os.path.abspath(file_path)

    @staticmethod
    def file_exists(file_path : str) -> bool:
        """
        Checks if a file exists at the specified path.

        Args:
            file_path (str): The path to the file.

        Returns:
            bool: True if the file exists, False otherwise.
        """
        return os.path.isfile(file_path)

    @staticmethod
    def create_directory(dir_path : str) -> None:
        """
        Creates a directory if it doesn't already exist.

        Args:
            dir_path (str): The path of the directory to create.
        """
        os.makedirs(dir_path, exist_ok=True)
