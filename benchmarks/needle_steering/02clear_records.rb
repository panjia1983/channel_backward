require_relative 'model'
Record.where(version: 31).delete_all
