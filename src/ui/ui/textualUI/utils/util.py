from dotenv import dotenv_values


def is_dev() -> bool:
    """
    Check if the environment is set to development.

    :return: True if the environment is set to development, False otherwise
    """
    config_data = dotenv_values("src/raspi/.env")
    if config_data is None:
        raise ValueError("No config file found. Create a .env file in src/raspi")
    return config_data.get("ENV") == "dev"
