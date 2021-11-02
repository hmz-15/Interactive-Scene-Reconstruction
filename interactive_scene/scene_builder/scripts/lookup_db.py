from db_loader import DbLoader

def foo(db_dir, config_file):
    db = DbLoader(db_dir, config_file)
    print(db)

    print(db.random_pick("Keyboard"))


if __name__ == "__main__":
    db_dir = "assets/rigid/"
    config_file = db_dir + "cad_models_rigid.csv"

    foo(db_dir, config_file)