require_relative 'model'
Record.where(version: 32).delete_all
