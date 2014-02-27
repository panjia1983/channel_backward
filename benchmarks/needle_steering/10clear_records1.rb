require_relative 'model'
Record.where(version: 11001).delete_all
