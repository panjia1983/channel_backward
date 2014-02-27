require_relative 'model'
Record.where(version: 10502).delete_all
