require_relative 'model'
Record.where(version: 10602).delete_all
